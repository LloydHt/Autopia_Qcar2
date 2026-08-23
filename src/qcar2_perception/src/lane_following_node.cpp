#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>

class LaneFollowingNode : public rclcpp::Node
{
public:
  LaneFollowingNode() : Node("lane_following_node"),
    prev_error_(0.0),
    integral_(0.0),
    last_error_(0.0),
    lost_frames_(0),
    log_counter_(0)
  {
    // PARAMÈTRES (Adoucis pour donner moins de "pouvoir" à la caméra)
    this->declare_parameter("kp",            0.004); // Divisé par 2 (était 0.008)
    this->declare_parameter("ki",            0.000);
    this->declare_parameter("kd",            0.004); // Divisé par 2
    this->declare_parameter("base_speed",    0.15);
    this->declare_parameter("roi_ratio",     0.40);
    this->declare_parameter("min_area",      200.0);
    this->declare_parameter("target_ratio",  0.3);

    // SUBSCRIBERS
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&LaneFollowingNode::imageCallback, this, std::placeholders::_1)
    );

    // PUBLISHERS
    cmd_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("/lane_cmd_vel", 10);
    error_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lane_error", 10);

    RCLCPP_INFO(this->get_logger(), "Lane Following Node démarré ✅ (Mode Ligne Jaune Uniquement - Herbe désactivée)");
  }

  ~LaneFollowingNode() { stopRobot(); }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) { return; }

    cv::Mat frame = cv_ptr->image;
    if (frame.empty()) return;

    double kp           = this->get_parameter("kp").as_double();
    double ki           = this->get_parameter("ki").as_double();
    double kd           = this->get_parameter("kd").as_double();
    double base_speed   = this->get_parameter("base_speed").as_double();
    double roi_ratio    = this->get_parameter("roi_ratio").as_double();
    double min_area     = this->get_parameter("min_area").as_double();
    double target_ratio = this->get_parameter("target_ratio").as_double();

    int roi_top = static_cast<int>(frame.rows * roi_ratio);
    cv::Mat roi = frame(cv::Rect(0, roi_top, frame.cols, frame.rows - roi_top));

    // Détection de la ligne jaune
    double error = detectYellowLane(roi, frame.cols, min_area, target_ratio);

    // Contrôleur PID classique
    double steering = computePID(error, kp, ki, kd);
    double speed    = base_speed * (1.0 - std::abs(steering) * 0.5); // Ralentit moins fort dans les virages
    speed           = std::max(speed, 0.05);

    publishCmd(speed, steering);

    // Publier l'erreur pour les métriques de l'article !
    auto error_msg = std_msgs::msg::Float32();
    error_msg.data = static_cast<float>(error);
    error_pub_->publish(error_msg);

    log_counter_++;
    if (log_counter_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "Erreur: %6.1f px | Steering: %5.3f | Speed: %.2f | Lost: %d",
        error, steering, speed, lost_frames_);
    }
  }

  double detectYellowLane(const cv::Mat& roi, int full_width, double min_area, double target_ratio)
  {
    cv::Mat hsv, mask;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(18, 80, 80), cv::Scalar(35, 255, 255), mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<int> cx_list;
    for (const auto& contour : contours) {
      if (cv::contourArea(contour) < min_area) continue;
      cv::Moments M = cv::moments(contour);
      if (M.m00 > 0) cx_list.push_back(static_cast<int>(M.m10 / M.m00));
    }

    int target_x = static_cast<int>(full_width * target_ratio);
    double error = 0.0;

    if (!cx_list.empty()) {
      double sum = std::accumulate(cx_list.begin(), cx_list.end(), 0.0);
      int lane_center = static_cast<int>(sum / cx_list.size());
      error = static_cast<double>(lane_center - target_x);
      last_error_  = error;
      lost_frames_ = 0;
    } else {
      lost_frames_++;
      // Si on perd la ligne, on réduit doucement l'erreur précédente pour ne pas faire d'à-coups
      error = last_error_ * 0.8; 
    }
    return error;
  }

  double computePID(double error, double kp, double ki, double kd)
  {
    integral_ += error;
    double derivative = error - prev_error_;
    prev_error_ = error;
    double output = kp * error + ki * integral_ + kd * derivative;
    return std::clamp(output, -0.3, 0.3); // Bridé à 0.3 rad/s au lieu de 0.5 pour réduire son pouvoir
  }

  void publishCmd(double speed, double steering)
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x  = speed;
    msg.angular.z = -steering;
    cmd_pub_->publish(msg);
  }

  void stopRobot()
  {
    auto msg = geometry_msgs::msg::Twist();
    cmd_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr     error_pub_;

  double prev_error_, integral_, last_error_;
  int lost_frames_, log_counter_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneFollowingNode>());
  rclcpp::shutdown();
  return 0;
}