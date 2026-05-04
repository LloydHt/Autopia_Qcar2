#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class CmdVelArbiter : public rclcpp::Node
{
public:
  CmdVelArbiter() : Node("cmd_vel_arbiter")
  {
    // =========================================================
    // PARAMÈTRES
    // =========================================================
    this->declare_parameter("timeout_ms", 500);
    // Seuil de vitesse angulaire au-delà duquel on considère 
    // que la HDMap veut forcer un virage (rad/s)
    this->declare_parameter("hdmap_turn_threshold", 0.25); 

    // =========================================================
    // SUBSCRIBERS COMMANDES
    // =========================================================
    lidar_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/lidar_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        lidar_cmd_ = *msg; lidar_last_time_ = this->now(); lidar_active_ = true;
      });

    stop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/stop_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        stop_cmd_ = *msg; stop_last_time_ = this->now(); stop_active_ = true;
      });

    lane_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/lane_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        lane_cmd_ = *msg; lane_last_time_ = this->now(); lane_active_ = true;
      });

    hdmap_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/hdmap_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        hdmap_cmd_ = *msg; hdmap_last_time_ = this->now(); hdmap_active_ = true;
      });

    // =========================================================
    // SUBSCRIBERS ÉTATS
    // =========================================================
    stop_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/stop_detected", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { stop_detected_ = msg->data; });

    rp_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/hdmap_in_roundabout", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { in_roundabout_ = msg->data; });

    // =========================================================
    // PUBLISHERS
    // =========================================================
    cmd_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",        10);
    state_pub_ = this->create_publisher<std_msgs::msg::Bool>      ("/arbiter/active", 10);

    timer_ = this->create_wall_timer(
      50ms, std::bind(&CmdVelArbiter::arbitrate, this));

    RCLCPP_INFO(this->get_logger(), "CMD VEL Arbitre demarre ✅");
    RCLCPP_INFO(this->get_logger(),
      "Priorites : LiDAR > STOP > Navigation (HDMap prioritaire sur virages) > Safety");
  }

private:

  // =========================================================
  // ARBITRAGE — 20 Hz
  // =========================================================
  void arbitrate()
  {
    int timeout_ms = this->get_parameter("timeout_ms").as_int();
    double turn_threshold = this->get_parameter("hdmap_turn_threshold").as_double();
    auto now = this->now();

    // Vérification des timeouts (Watchdog)
    checkTimeout(lidar_active_, lidar_last_time_, now, timeout_ms, "LiDAR");
    checkTimeout(stop_active_,  stop_last_time_,  now, timeout_ms, "STOP");
    checkTimeout(lane_active_,  lane_last_time_,  now, timeout_ms, "Lane");
    checkTimeout(hdmap_active_, hdmap_last_time_, now, timeout_ms, "HDMap");

    geometry_msgs::msg::Twist final_cmd;
    std::string active_source = "NONE";

    // ── PRIORITÉ 1 : LiDAR (Freinage urgence OU évitement)
    if (lidar_active_) {
      final_cmd     = lidar_cmd_;
      active_source = "LIDAR";

    // ── PRIORITÉ 2 : STOP sign
    } else if (stop_active_ && stop_detected_) {
      final_cmd     = stop_cmd_;
      active_source = "STOP";

    // ── PRIORITÉ 3 : Navigation (Fusion Caméra / Carte)
    } else if (lane_active_ || hdmap_active_) {
      auto cmd = geometry_msgs::msg::Twist();

      // Vitesse → HDMap toujours (connaît les limitations de vitesse)
      cmd.linear.x = hdmap_active_ ? hdmap_cmd_.linear.x : lane_cmd_.linear.x;

      // ★ INTELLIGENCE HDMAP : Détection des virages
      bool hdmap_wants_to_turn = (hdmap_active_ && std::abs(hdmap_cmd_.angular.z) > turn_threshold);

      // Choix de la direction (angular.z)
      if (in_roundabout_) {
        // Dans un rond-point, la caméra est souvent perdue, on force la carte
        cmd.angular.z = hdmap_cmd_.angular.z;
        active_source = "HDMAP_RP";

      } else if (hdmap_wants_to_turn || !lane_active_) {
        // Virage demandé par la carte OU ligne complètement perdue
        cmd.angular.z = hdmap_cmd_.angular.z;
        active_source = hdmap_wants_to_turn ? "HDMAP_TURN" : "HDMAP_ONLY";

      } else {
        // Ligne droite ou courbe légère : le Lane Following centre le robot
        cmd.angular.z = lane_cmd_.angular.z;
        active_source = "LANE+HDMAP";
      }

      final_cmd = cmd;

    // ── PRIORITÉ 4 : Safety stop (Aucune source disponible)
    } else {
      final_cmd     = makeStop();
      active_source = "SAFETY_STOP";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Aucun node actif — STOP securite");
    }

    // ── Publication des commandes
    cmd_pub_->publish(final_cmd);

    // ── Publication de l'état du robot
    std_msgs::msg::Bool state_msg;
    state_msg.data = (active_source != "SAFETY_STOP");
    state_pub_->publish(state_msg);

    // ── Log (1 Hz)
    log_counter_++;
    if (log_counter_ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "Arbitre -> [%s] | v=%.2f w=%.2f | "
        "LIDAR:%s STOP:%s LANE:%s HDMAP:%s RP:%s",
        active_source.c_str(),
        final_cmd.linear.x, final_cmd.angular.z,
        lidar_active_  ? "OK" : "--",
        stop_active_   ? "OK" : "--",
        lane_active_   ? "OK" : "--",
        hdmap_active_  ? "OK" : "--",
        in_roundabout_ ? "OUI" : "non");
    }
  }

  // ── Helpers
  geometry_msgs::msg::Twist makeStop() {
    return geometry_msgs::msg::Twist();
  }

  void checkTimeout(bool& active, rclcpp::Time& last_time,
                    const rclcpp::Time& now, int timeout_ms,
                    const std::string& name)
  {
    if (!active) return;
    if ((now - last_time).seconds() * 1000.0 > timeout_ms) {
      active = false;
      RCLCPP_WARN(this->get_logger(), "Node %s timeout — desactive", name.c_str());
    }
  }

  // ── Subscribers commandes
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr stop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr lane_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr hdmap_sub_;

  // ── Subscribers états
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rp_sub_;

  // ── Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr       state_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ── Commandes reçues
  geometry_msgs::msg::Twist lidar_cmd_, stop_cmd_, lane_cmd_, hdmap_cmd_;

  // ── États actifs
  bool lidar_active_ = false;
  bool stop_active_  = false;
  bool lane_active_  = false;
  bool hdmap_active_ = false;

  // ── États détections
  bool stop_detected_ = false;
  bool in_roundabout_ = false;

  // ── Timestamps
  rclcpp::Time lidar_last_time_, stop_last_time_, lane_last_time_, hdmap_last_time_;

  int log_counter_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelArbiter>());
  rclcpp::shutdown();
  return 0;
}