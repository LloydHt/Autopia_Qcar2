#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class StopSignNode : public rclcpp::Node
{
public:
  StopSignNode() : Node("stop_sign_node"),
    stop_active_(false),
    log_counter_(0),
    hdmap_dist_(999.0f) // Initialisé très loin par défaut
  {
    // Paramètres
    this->declare_parameter("stop_duration",     3.0);   // Durée arrêt
    this->declare_parameter("cooldown_duration", 4.0);   // Temps aveugle après l'arrêt
    this->declare_parameter("slow_speed",        0.1);  // Vitesse ralenti
    this->declare_parameter("min_sides",         6);
    this->declare_parameter("max_sides",         10);
    this->declare_parameter("right_ratio",       0.40);

    // Subscribers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&StopSignNode::imageCallback, this, std::placeholders::_1)
    );

    // ★ NOUVEAU : On écoute la distance calculée par la HD Map
    hdmap_dist_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/hdmap_stop_distance", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        hdmap_dist_ = msg->data;
      }
    );

    // Publishers
    cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/stop_cmd_vel", 10);
    stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/stop_detected", 10);
    dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("/stop_distance", 10);

    RCLCPP_INFO(this->get_logger(), "Stop Sign Node démarré ✅ (Mode Fusion : Vision + Carte)");
  }

private:

  enum class State {
    DRIVING,
    SLOWING,
    WAITING,
    COOLDOWN
  };

  State state_ = State::DRIVING;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    double stop_dur     = this->get_parameter("stop_duration").as_double();
    double cooldown_dur = this->get_parameter("cooldown_duration").as_double();
    double slow_speed   = this->get_parameter("slow_speed").as_double();

    // =========================================================
    // 1. SI ON ATTEND (WAITING)
    // =========================================================
    if (state_ == State::WAITING) {
      publishStop();
      if ((this->now() - stop_start_time_).seconds() >= stop_dur) {
        state_ = State::COOLDOWN;
        cooldown_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "✅ Fin du stop (3s). Passage en mode Cooldown pour %gs", cooldown_dur);
        
        // On libère l'arbitre
        auto stop_msg = std_msgs::msg::Bool();
        stop_msg.data = false;
        stop_pub_->publish(stop_msg);
      }
      return;
    }

    // =========================================================
    // 2. SI ON EST EN COOLDOWN (Pour dépasser le panneau)
    // =========================================================
    if (state_ == State::COOLDOWN) {
      if ((this->now() - cooldown_start_time_).seconds() >= cooldown_dur) {
        state_ = State::DRIVING;
        RCLCPP_INFO(this->get_logger(), "👁️ Fin du Cooldown, détection réactivée.");
      }
      return; // On ignore l'image pendant le cooldown
    }

    // =========================================================
    // 3. ANALYSE DE L'IMAGE (OpenCV)
    // =========================================================
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) { return; }

    cv::Mat frame = cv_ptr->image;
    if (frame.empty()) return;

    int roi_bottom = static_cast<int>(frame.rows * 0.75);
    cv::Mat roi = frame(cv::Rect(0, 0, frame.cols, roi_bottom));

    double best_area = 0.0;
    double best_cx   = 0.0;
    bool   found     = false;

    int min_sides   = this->get_parameter("min_sides").as_int();
    int max_sides   = this->get_parameter("max_sides").as_int();
    double right_r  = this->get_parameter("right_ratio").as_double();

    detectStopSign(roi, frame.cols, min_sides, max_sides, right_r, best_area, best_cx, found);

    // =========================================================
    // 4. LOGIQUE DE DÉCISION (FUSION VISION + CARTE)
    // =========================================================
    auto stop_msg  = std_msgs::msg::Bool();
    auto dist_msg  = std_msgs::msg::Float32();
    dist_msg.data  = hdmap_dist_; // On publie la vraie distance de la carte

    if (!found) {
      // Aucun panneau en vue
      state_        = State::DRIVING;
      stop_msg.data = false;
    } else {
      // PANNEAU VU ! On vérifie la distance fournie par la HD Map
      // Le seuil est 0.35m car c'est le rayon d'acceptation (WAYPOINT_RADIUS) de la HD Map
      if (hdmap_dist_ <= 0.35f && hdmap_dist_ >= 0.0f) {
        
        // On est sur la ligne + le panneau est validé !
        state_           = State::WAITING;
        stop_start_time_ = this->now();
        stop_msg.data    = true;
        publishStop();
        RCLCPP_INFO(this->get_logger(), "🛑 ARRÊT FUSION ! Panneau détecté ET Ligne atteinte (%.2fm)", hdmap_dist_);
        
      } else if (hdmap_dist_ <= 0.6f) { // À moins de 0.6m, on commence à ralentir
        
        state_        = State::SLOWING;
        stop_msg.data = true;
        publishSlow(slow_speed);
        
      } else {
        // Panneau vu mais on est encore trop loin
        state_        = State::DRIVING;
        stop_msg.data = false;
      }
    }

    stop_pub_->publish(stop_msg);
    dist_pub_->publish(dist_msg);

    log_counter_++;
    if (log_counter_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), "State: %d | Found: %s | Area: %.0f | MapDist: %.2fm", 
        (int)state_, found ? "YES" : "NO", best_area, hdmap_dist_);
    }
  }

  // ── OpenCV Helper
  void detectStopSign(const cv::Mat& roi, int full_width, int min_sides, int max_sides,
                      double right_ratio, double& best_area, double& best_cx, bool& found)
  {
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(165, 100, 50), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int right_threshold = static_cast<int>(full_width * right_ratio);

    for (const auto& contour : contours) {
      double area = cv::contourArea(contour);
      if (area < 300.0) continue;

      std::vector<cv::Point> approx;
      double epsilon = 0.04 * cv::arcLength(contour, true);
      cv::approxPolyDP(contour, approx, epsilon, true);

      int sides = static_cast<int>(approx.size());
      if (sides < min_sides || sides > max_sides) continue;

      cv::Moments M = cv::moments(contour);
      if (M.m00 <= 0) continue;
      double cx = M.m10 / M.m00;

      if (cx < right_threshold) continue;

      if (area > best_area) {
        best_area = area;
        best_cx   = cx;
        found     = true;
      }
    }
  }

  void publishStop()
  {
    auto msg = geometry_msgs::msg::Twist();
    cmd_pub_->publish(msg);
  }

  void publishSlow(double speed)
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = speed;
    cmd_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  hdmap_dist_sub_; // NOUVEAU
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr     dist_pub_;

  bool stop_active_;
  int log_counter_;
  float hdmap_dist_; // NOUVEAU

  rclcpp::Time stop_start_time_;
  rclcpp::Time cooldown_start_time_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StopSignNode>());
  rclcpp::shutdown();
  return 0;
}