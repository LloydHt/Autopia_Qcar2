#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>

// =========================================================
// Structure résultat détection herbe par zone
// =========================================================
struct GrassDetection {
  bool left;    // Herbe à gauche ?
  bool right;   // Herbe à droite ?
  bool center;  // Herbe au centre ?
};

class LaneFollowingNode : public rclcpp::Node
{
public:
  LaneFollowingNode() : Node("lane_following_node"),
    prev_error_(0.0),
    integral_(0.0),
    last_error_(0.0),
    lost_frames_(0),
    active_(true),
    log_counter_(0)
  {
    // =========================================================
    // PARAMÈTRES — Modifiables sans recompiler
    // ros2 param set /lane_following_node kp 0.01
    // =========================================================
    this->declare_parameter("kp",            0.008);
    this->declare_parameter("ki",            0.000);
    this->declare_parameter("kd",            0.008);
    this->declare_parameter("base_speed",    0.15);
    this->declare_parameter("roi_ratio",     0.40);  // 60% bas de l'image
    this->declare_parameter("min_area",      200.0); // Aire min contour (px²)
    this->declare_parameter("target_ratio",  0.3);  // Ligne jaune à 25% gauche
    this->declare_parameter("grass_thresh",  0.15);  // Seuil détection herbe
    this->declare_parameter("turn_speed",    0.07);  // Vitesse pendant virage herbe
    this->declare_parameter("turn_angle",    0.35);  // Angle virage herbe (rad/s)

    // =========================================================
    // SUBSCRIBERS
    // =========================================================
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&LaneFollowingNode::imageCallback, this, std::placeholders::_1)
    );

    // =========================================================
    // PUBLISHERS
    // =========================================================
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/lane_cmd_vel", 10
    );

    // Erreur latérale → données article
    error_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/lane_error", 10
    );

    // Détection herbe → données article
    grass_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/grass_detected", 10
    );

    RCLCPP_INFO(this->get_logger(), "Lane Following Node démarré ✅");
    RCLCPP_INFO(this->get_logger(), "Mode : PID ligne jaune + herbe en fallback");
  }

  ~LaneFollowingNode()
  {
    stopRobot();
  }

private:

  // =========================================================
  // CALLBACK PRINCIPAL — ~18 Hz
  // =========================================================
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    if (frame.empty()) return;

    // --- Paramètres ---
    double kp           = this->get_parameter("kp").as_double();
    double ki           = this->get_parameter("ki").as_double();
    double kd           = this->get_parameter("kd").as_double();
    double base_speed   = this->get_parameter("base_speed").as_double();
    double roi_ratio    = this->get_parameter("roi_ratio").as_double();
    double min_area     = this->get_parameter("min_area").as_double();
    double target_ratio = this->get_parameter("target_ratio").as_double();
    double grass_thresh = this->get_parameter("grass_thresh").as_double();
    double turn_speed   = this->get_parameter("turn_speed").as_double();
    double turn_angle   = this->get_parameter("turn_angle").as_double();

    // --- ROI : bas de l'image ---
    int roi_top = static_cast<int>(frame.rows * roi_ratio);
    cv::Mat roi = frame(cv::Rect(0, roi_top,
                                  frame.cols,
                                  frame.rows - roi_top));

    // =========================================================
    // NAVIGATION PRINCIPALE
    // detectYellowLane gère tout :
    //   → Lignes visibles  : PID (herbe ignorée)
    //   → Lignes perdues   : fallback herbe
    // =========================================================
    double error = detectYellowLane(roi, frame.cols,
                                     min_area, target_ratio,
                                     grass_thresh, turn_speed, turn_angle);

    // Si detectYellowLane a déjà publié une commande herbe → ne pas écraser
    if (grass_active_) {
      // La commande herbe a déjà été publiée dans detectYellowLane
      // Juste publier l'erreur et logger
    } else {
      // Mode normal — PID
      double steering = computePID(error, kp, ki, kd);
      double speed    = base_speed * (1.0 - std::abs(steering) * 0.8);
      speed           = std::max(speed, 0.05);

      if (active_) {
        publishCmd(speed, steering);
      }
    }

    // Publier erreur latérale (données article)
    auto error_msg = std_msgs::msg::Float32();
    error_msg.data = static_cast<float>(error);
    error_pub_->publish(error_msg);

    // Publier état herbe (données article)
    auto grass_msg = std_msgs::msg::Bool();
    grass_msg.data = grass_active_;
    grass_pub_->publish(grass_msg);

    // Log terminal (1 fois sur 5)
    log_counter_++;
    if (log_counter_ % 5 == 0) {
      if (grass_active_) {
        RCLCPP_INFO(this->get_logger(),
          "🌿 MODE HERBE | Lost: %d frames", lost_frames_);
      } else {
        double steering = computePID(error, kp, ki, kd);
        double speed    = base_speed * (1.0 - std::abs(steering) * 0.8);
        RCLCPP_INFO(this->get_logger(),
          "Erreur: %6.1f px | Steering: %5.3f | Speed: %.2f | Lost: %d",
          error, steering, speed, lost_frames_);
      }
    }
  }

  // =========================================================
  // DÉTECTION HERBE PAR ZONES
  //
  // Divise le ROI en 3 zones horizontales :
  // ┌──────────┬──────────┬──────────┐
  // │  GAUCHE  │  CENTRE  │  DROITE  │
  // │   0-33%  │  33-66%  │  66-100% │
  // └──────────┴──────────┴──────────┘
  // =========================================================
  GrassDetection detectGrassZones(const cv::Mat& roi, double threshold)
  {
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // Vert herbe : H=35-85, S=50-255, V=50-255
    cv::Scalar lower_green(35,  50,  50);
    cv::Scalar upper_green(85, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv, lower_green, upper_green, mask);

    int w     = mask.cols;
    int total = mask.rows * (w / 3);

    // Zone gauche : 0 → w/3
    cv::Mat zone_left   = mask(cv::Rect(0,     0, w/3, mask.rows));
    // Zone centre : w/3 → 2w/3
    cv::Mat zone_center = mask(cv::Rect(w/3,   0, w/3, mask.rows));
    // Zone droite : 2w/3 → w
    cv::Mat zone_right  = mask(cv::Rect(2*w/3, 0, w/3, mask.rows));

    GrassDetection result;
    result.left   = (cv::countNonZero(zone_left)   / (double)total) > threshold;
    result.center = (cv::countNonZero(zone_center) / (double)total) > threshold;
    result.right  = (cv::countNonZero(zone_right)  / (double)total) > threshold;

    return result;
  }

  // =========================================================
  // DÉTECTION LIGNES JAUNES + FALLBACK HERBE
  //
  // Priorités :
  //   1. Lignes jaunes visibles → PID (herbe ignorée)
  //   2. lost < 15  → last_error * 0.8
  //   3. lost < 30  → last_error * 0.3
  //   4. lost >= 30 → Détection herbe pour guider le virage
  // =========================================================
  double detectYellowLane(const cv::Mat& roi,
                           int full_width,
                           double min_area,
                           double target_ratio,
                           double grass_thresh,
                           double turn_speed,
                           double turn_angle)
  {
    // --- BGR → HSV ---
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // --- Isoler le jaune ---
    cv::Scalar lower_yellow(18,  80,  80);
    cv::Scalar upper_yellow(35, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv, lower_yellow, upper_yellow, mask);

    // --- Morphologie ---
    cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // --- Trouver les contours ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours,
                     cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // --- Centroïdes ---
    std::vector<int> cx_list;
    for (const auto& contour : contours) {
      if (cv::contourArea(contour) < min_area) continue;
      cv::Moments M = cv::moments(contour);
      if (M.m00 <= 0) continue;
      cx_list.push_back(static_cast<int>(M.m10 / M.m00));
    }

    // Position cible de la ligne jaune
    // 25% depuis la gauche → robot reste à gauche de la ligne
    int target_x = static_cast<int>(full_width * target_ratio);

    double error = 0.0;

    // ─────────────────────────────────────────────────────
    // CAS 1 : Lignes jaunes visibles → PID normal
    //         Herbe complètement ignorée ici
    // ─────────────────────────────────────────────────────
    if (!cx_list.empty()) {
      double sum = std::accumulate(cx_list.begin(), cx_list.end(), 0.0);
      int lane_center = static_cast<int>(sum / cx_list.size());

      error        = static_cast<double>(lane_center - target_x);
      last_error_  = error;
      lost_frames_ = 0;
      grass_active_ = false;  // Désactiver mode herbe

    // ─────────────────────────────────────────────────────
    // CAS 2 : Lignes perdues < 15 frames (~0.8s)
    //         Maintenir dernière correction atténuée
    // ─────────────────────────────────────────────────────
    } else if (lost_frames_ < 10) {
      lost_frames_++;
      error         = last_error_ * 0.8;
      grass_active_ = false;
      RCLCPP_WARN(this->get_logger(),
        "Lignes perdues [%d/15] — maintien: %.1f px",
        lost_frames_, error);

    // ─────────────────────────────────────────────────────
    // CAS 3 : Lignes perdues 15-30 frames (~1.6s)
    //         Réduire encore la correction
    // ─────────────────────────────────────────────────────
    } else if (lost_frames_ < 15) {
      lost_frames_++;
      error         = last_error_ * 0.3;
      grass_active_ = false;
      RCLCPP_WARN(this->get_logger(),
        "Lignes perdues [%d/30] — recherche...", lost_frames_);

    // ─────────────────────────────────────────────────────
    // CAS 4 : Lignes perdues >= 30 frames
    //         → Activer détection herbe pour guider le virage
    //         C'est ici qu'on utilise l'herbe — PAS AVANT
    // ─────────────────────────────────────────────────────
    } else {
      lost_frames_++;
      grass_active_ = true;
      error         = 0.0;

      GrassDetection grass = detectGrassZones(roi, grass_thresh);

      if (grass.left && !grass.right) {
        // Herbe à gauche seulement → mur vert gauche → tourner à droite
        publishCmd(turn_speed, -turn_angle);
        RCLCPP_INFO(this->get_logger(),
          "🌿 Virage : herbe GAUCHE → tourner DROITE (lost: %d)", lost_frames_);

      } else if (grass.right && !grass.left) {
        // Herbe à droite seulement → mur vert droit → tourner à gauche
        publishCmd(turn_speed, turn_angle);
        RCLCPP_INFO(this->get_logger(),
          "🌿 Virage : herbe DROITE → tourner GAUCHE (lost: %d)", lost_frames_);

      } else if (grass.left && grass.right) {
        // Herbe des deux côtés → avancer doucement, chercher la ligne
        publishCmd(0.05, 0.0);
        RCLCPP_WARN(this->get_logger(),
          "🌿 Herbe partout — avance doucement... (lost: %d)", lost_frames_);

      } else {
        // Pas de ligne ET pas d'herbe → continuer tout droit doucement
        publishCmd(0.1, 0.0);
        RCLCPP_WARN(this->get_logger(),
          "⚠️ Rien détecté — avance tout droit (lost: %d)", lost_frames_);
      }
    }

    return error;
  }

  // =========================================================
  // CONTRÔLEUR PID
  // =========================================================
  double computePID(double error, double kp, double ki, double kd)
  {
    integral_ += error;
    double derivative = error - prev_error_;
    prev_error_ = error;
    double output = kp * error + ki * integral_ + kd * derivative;
    return std::clamp(output, -0.5, 0.5);
  }

  // =========================================================
  // COMMANDES
  // =========================================================
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
    msg.linear.x  = 0.0;
    msg.angular.z = 0.0;
    cmd_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Robot arrêté ⛔");
  }

  // =========================================================
  // MEMBRES PRIVÉS
  // =========================================================

  // ROS2
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr     error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        grass_pub_;

  // PID
  double prev_error_;
  double integral_;

  // Mémoire perte de ligne
  double last_error_;
  int    lost_frames_;
  bool   grass_active_;  // true = mode herbe actif

  // Divers
  bool active_;
  int  log_counter_;
};

// =========================================================
// MAIN
// =========================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneFollowingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}