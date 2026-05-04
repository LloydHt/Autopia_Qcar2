#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>

#include "qcar2_perception/autopia_hdmap.hpp"

class HDMapNavigationNode : public rclcpp::Node
{
public:
  HDMapNavigationNode() : Node("hdmap_navigation_node"),
    robot_x_(0.0), robot_y_(0.0), robot_yaw_(0.0),
    odom_received_(false), log_counter_(0)
  {
    this->declare_parameter("mission",          std::string("test_ligne_droite"));
    this->declare_parameter("waypoint_radius",  0.4);
    this->declare_parameter("kp_angular",       1.2);
    this->declare_parameter("max_angular",      0.4);
    this->declare_parameter("stop_ahead_dist",  3.0);

    std::string mission = this->get_parameter("mission").as_string();
    if (!hdmap_.loadMission(mission)) {
      RCLCPP_ERROR(this->get_logger(), "Mission inconnue : %s", mission.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),
      "Mission chargee : %s (%d waypoints)",
      mission.c_str(), hdmap_.getMissionSize());

    // ── SUBSCRIBER odométrie
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
          std::bind(&HDMapNavigationNode::odomCallback, this,
          std::placeholders::_1)
    );

    // ── PUBLISHERS
    cmd_pub_        = this->create_publisher<geometry_msgs::msg::Twist>("/hdmap_cmd_vel",      10);
    stop_dist_pub_  = this->create_publisher<std_msgs::msg::Float32>   ("/hdmap_stop_distance",10);
    stop_ahead_pub_ = this->create_publisher<std_msgs::msg::Bool>      ("/hdmap_stop_ahead",   10);
    node_pub_       = this->create_publisher<std_msgs::msg::String>    ("/hdmap_current_node", 10);
    // ★ NOUVEAU — signale au round-point que l'on est dans le rond-point
    rp_pub_         = this->create_publisher<std_msgs::msg::Bool>      ("/hdmap_in_roundabout",10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HDMapNavigationNode::navigate, this)
    );

    RCLCPP_INFO(this->get_logger(), "HD Map Navigation Node demarre ✅");
  }

  ~HDMapNavigationNode() { publishStop(); }

private:

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    auto& q  = msg->pose.pose.orientation;
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    robot_yaw_  = std::atan2(siny, cosy);
    odom_received_ = true;
  }

  void navigate()
  {
    if (!odom_received_) return;

    // ── Mission terminée
    if (hdmap_.isMissionComplete()) {
      publishStop();
      publishRoundabout(false);
      RCLCPP_INFO_ONCE(this->get_logger(), "Mission terminee !");
      return;
    }

    const HDMapNode* wp = hdmap_.getCurrentWaypoint();
    if (!wp) return;

    // ★ NOUVEAU — publier état rond-point AVANT tout
    bool in_rp = (wp->node_type == "roundabout");
    publishRoundabout(in_rp);

    double kp          = this->get_parameter("kp_angular").as_double();
    double max_angular = this->get_parameter("max_angular").as_double();
    double stop_dist   = this->get_parameter("stop_ahead_dist").as_double();

    double dist_wp = hdmap_.getDistanceToCurrent(robot_x_, robot_y_);

    // ── Waypoint atteint
    if (hdmap_.isWaypointReached(robot_x_, robot_y_)) {
      RCLCPP_INFO(this->get_logger(), "Waypoint [%s] atteint !", wp->node_id.c_str());
      hdmap_.advanceWaypoint();
      return;
    }

    // ── Calcul commande
    double heading     = hdmap_.getHeadingToCurrent(robot_x_, robot_y_);
    double angle_error = normalizeAngle(heading - robot_yaw_);
    double angular_z   = std::clamp(kp * angle_error, -max_angular, max_angular);
    double speed       = hdmap_.getCurrentSpeedLimit();

    // ★ Dans le rond-point : augmenter kp pour tourner plus précisément
    if (in_rp) {
      double kp_rp   = kp * 1.5;
      angular_z      = std::clamp(kp_rp * angle_error, -max_angular, max_angular);
    }

    auto cmd      = geometry_msgs::msg::Twist();
    cmd.linear.x  = speed;
    cmd.angular.z = angular_z;
    cmd_pub_->publish(cmd);

    // ── Distance prochain STOP
    auto [dist_stop, has_stop] = hdmap_.getDistanceToNextStop(robot_x_, robot_y_);

    auto dist_msg  = std_msgs::msg::Float32();
    dist_msg.data  = static_cast<float>(dist_stop);
    stop_dist_pub_->publish(dist_msg);

    auto ahead_msg = std_msgs::msg::Bool();
    ahead_msg.data = has_stop && (dist_stop < stop_dist);
    stop_ahead_pub_->publish(ahead_msg);

    auto node_msg  = std_msgs::msg::String();
    node_msg.data  = wp->node_id;
    node_pub_->publish(node_msg);

    // ── Log 1 Hz
    log_counter_++;
    if (log_counter_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "-> [%s] dist=%.2fm err=%.1fdeg v=%.2f w=%.2f stop=%.1fm %s",
        wp->node_id.c_str(),
        dist_wp,
        angle_error * 180.0 / M_PI,
        speed, angular_z, dist_stop,
        in_rp ? "[ROND-POINT]" : "");
    }
  }

  // ── Helpers
  double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  void publishStop() {
    geometry_msgs::msg::Twist msg;
    cmd_pub_->publish(msg);
  }

  void publishRoundabout(bool val) {
    std_msgs::msg::Bool msg;
    msg.data = val;
    rp_pub_->publish(msg);
  }

  // ── Membres
  AutopiaHDMap hdmap_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr     stop_dist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        stop_ahead_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      node_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        rp_pub_;   // ★ NOUVEAU
  rclcpp::TimerBase::SharedPtr timer_;

  double robot_x_, robot_y_, robot_yaw_;
  bool   odom_received_;
  int    log_counter_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HDMapNavigationNode>());
  rclcpp::shutdown();
  return 0;
}