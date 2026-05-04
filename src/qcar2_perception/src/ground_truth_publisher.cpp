#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GroundTruthPublisher : public rclcpp::Node
{
public:
  GroundTruthPublisher() : Node("ground_truth_publisher")
  {
    // Subscriber — Gazebo dynamic pose
    pose_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/world/autopia_world/dynamic_pose/info", 10,
      std::bind(&GroundTruthPublisher::poseCallback, this,
                std::placeholders::_1)
    );

    // Publisher — position propre pour HD Map
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/ground_truth", 10
    );

    RCLCPP_INFO(this->get_logger(),
      "Ground Truth Publisher demarre ✅");
    RCLCPP_INFO(this->get_logger(),
      "Publie sur /ground_truth (coordonnees absolues Gazebo)");
  }

private:
  void poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    if (msg->transforms.empty()) return;

    // Le premier transform est toujours le qcar2
    auto& t = msg->transforms[0].transform;

    // Extraire yaw depuis quaternion
    auto& q     = t.rotation;
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw  = std::atan2(siny, cosy);

    // Publier comme Odometry (même format que /odometry/filtered)
    auto odom          = nav_msgs::msg::Odometry();
    odom.header.stamp  = this->now();
    odom.header.frame_id    = "qcar2/odom";
    odom.child_frame_id     = "qcar2/base_link";

    odom.pose.pose.position.x    = t.translation.x;
    odom.pose.pose.position.y    = t.translation.y;
    odom.pose.pose.position.z    = t.translation.z;
    odom.pose.pose.orientation.x = q.x;
    odom.pose.pose.orientation.y = q.y;
    odom.pose.pose.orientation.z = q.z;
    odom.pose.pose.orientation.w = q.w;

    odom_pub_->publish(odom);

    // Log 1 Hz
    log_counter_++;
    if (log_counter_ % 50 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "GT pos: x=%.3f y=%.3f yaw=%.1f°",
        t.translation.x, t.translation.y,
        yaw * 180.0 / M_PI);
    }
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr     odom_pub_;
  int log_counter_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthPublisher>());
  rclcpp::shutdown();
  return 0;
}