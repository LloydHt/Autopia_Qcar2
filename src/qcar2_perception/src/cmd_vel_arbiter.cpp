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
    // On passe le timeout à 1.5 secondes pour tolérer les sauts de waypoints
    this->declare_parameter("timeout_ms", 1500);

    lidar_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/lidar_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { lidar_cmd_ = *msg; lidar_last_time_ = this->now(); lidar_active_ = true; });

    stop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/stop_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { stop_cmd_ = *msg; stop_last_time_ = this->now(); stop_active_ = true; });

    lane_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/lane_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { lane_cmd_ = *msg; lane_last_time_ = this->now(); lane_active_ = true; });

    hdmap_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/hdmap_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { 
        hdmap_cmd_ = *msg; 
        hdmap_last_time_ = this->now(); 
        hdmap_active_ = true; 
        hdmap_ever_active_ = true; // NOUVEAU : On sait que la carte a démarré au moins une fois
      });

    stop_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>("/stop_detected", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { stop_detected_ = msg->data; });

    rp_sub_ = this->create_subscription<std_msgs::msg::Bool>("/hdmap_in_roundabout", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { in_roundabout_ = msg->data; });

    cmd_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arbiter/active", 10);

    // NOUVEAU : create_timer (horloge ROS) au lieu de create_wall_timer (horloge PC)
    timer_ = this->create_timer(50ms, std::bind(&CmdVelArbiter::arbitrate, this));

    RCLCPP_INFO(this->get_logger(), "CMD VEL Arbitre demarre ✅ (Mode Shadow Strict - Horloge Sim synchronisee)");
  }

private:
  void arbitrate()
  {
    int timeout_ms = this->get_parameter("timeout_ms").as_int();
    auto now = this->now();

    checkTimeout(lidar_active_, lidar_last_time_, now, timeout_ms, "LiDAR");
    checkTimeout(stop_active_,  stop_last_time_,  now, timeout_ms, "STOP");
    checkTimeout(lane_active_,  lane_last_time_,  now, timeout_ms, "Lane");
    checkTimeout(hdmap_active_, hdmap_last_time_, now, timeout_ms, "HDMap");

    geometry_msgs::msg::Twist final_cmd;
    std::string active_source = "NONE";

    // ── PRIORITÉ 1 : LiDAR
    if (lidar_active_) {
      final_cmd     = lidar_cmd_;
      active_source = "LIDAR_AEB";

    // ── PRIORITÉ 2 : STOP sign
    } else if (stop_active_ && stop_detected_) {
      final_cmd     = stop_cmd_;
      active_source = "STOP_SIGN";

    // ── PRIORITÉ 3 : HD Map
    } else if (hdmap_active_) {
      final_cmd.linear.x  = hdmap_cmd_.linear.x;
      final_cmd.angular.z = hdmap_cmd_.angular.z; 
      active_source = in_roundabout_ ? "HDMAP_RP" : "HDMAP_NOMINAL";

    // ── PRIORITÉ 4 : Fallback Caméra
    } else if (lane_active_) {
      final_cmd.linear.x  = lane_cmd_.linear.x;
      final_cmd.angular.z = lane_cmd_.angular.z;
      active_source = "LANE_FALLBACK";
      
      // LOGIQUE D'AFFICHAGE INTELLIGENTE
      if (hdmap_ever_active_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
          "⚠️ HD Map PERDUE (Timeout) ! Mode survie caméra activé.");
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
          "⏳ Initialisation... En attente de la HD Map.");
      }

    // ── PRIORITÉ 5 : Safety stop
    } else {
      final_cmd     = geometry_msgs::msg::Twist();
      active_source = "SAFETY_STOP";
    }

    cmd_pub_->publish(final_cmd);

    std_msgs::msg::Bool state_msg;
    state_msg.data = (active_source != "SAFETY_STOP");
    state_pub_->publish(state_msg);

    log_counter_++;
    if (log_counter_ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "Arbitre -> [%s] | v=%.2f w=%.2f",
        active_source.c_str(), final_cmd.linear.x, final_cmd.angular.z);
    }
  }

  void checkTimeout(bool& active, rclcpp::Time& last_time, const rclcpp::Time& now, int timeout_ms, const std::string& name) {
    if (!active) return;
    if ((now - last_time).seconds() * 1000.0 > timeout_ms) {
      active = false;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr lidar_sub_, stop_sub_, lane_sub_, hdmap_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_detected_sub_, rp_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist lidar_cmd_, stop_cmd_, lane_cmd_, hdmap_cmd_;
  bool lidar_active_ = false, stop_active_ = false, lane_active_ = false, hdmap_active_ = false;
  bool hdmap_ever_active_ = false; // NOUVEAU
  bool stop_detected_ = false, in_roundabout_ = false;
  rclcpp::Time lidar_last_time_, stop_last_time_, lane_last_time_, hdmap_last_time_;
  int log_counter_ = 0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelArbiter>());
  rclcpp::shutdown();
  return 0;
}