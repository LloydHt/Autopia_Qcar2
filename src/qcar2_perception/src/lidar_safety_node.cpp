#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

// On crée une classe qui "hérite" de rclcpp::Node
class LidarSafety : public rclcpp::Node {
public:
    LidarSafety() : Node("lidar_safety_node") {
        // 1. Abonnement au LiDAR (On écoute ce que le capteur dit)
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarSafety::scan_callback, this, std::placeholders::_1));
        
        // 2. Publication sur /cmd_vel (On prend le contrôle des moteurs)
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/lidar_cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "--- Système de Sécurité LiDAR Activé (C++) ---");
    }

private:
    // Cette fonction est appelée 10 fois par seconde (fréquence du LiDAR)
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        bool obstacle_detecte = false;
        
        // On définit la zone de danger (le "cône" devant la voiture)
        // Le LiDAR 360° commence souvent derrière (index 0). 
        // L'avant se situe généralement au milieu du tableau.
        
        int num_points = msg->ranges.size();
        for (int i = 0; i < num_points; i++) {
            float distance = msg->ranges[i];
            
            // On calcule l'angle du point actuel (en radians)
            float angle = msg->angle_min + (i * msg->angle_increment);

            // LOGIQUE : Si l'angle est entre -30° (-0.5 rad) et +30° (0.5 rad)
            // ET que la distance est inférieure à 0.6 mètre
            if (angle > -0.5 && angle < 0.5) {
                if (distance < 0.3 && distance > 0.05) {
                    obstacle_detecte = true;
                    break; 
                }
            }
        }

        if (obstacle_detecte) {
            RCLCPP_WARN(this->get_logger(), "!! DANGER !! Obstacle devant. Freinage !");
            auto stop_msg = geometry_msgs::msg::Twist();
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            publisher_->publish(stop_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSafety>());
    rclcpp::shutdown();
    return 0;
}