#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoider : public rclcpp::Node
{
public:
    ObstacleAvoider() : Node("obstacle_avoider")
    {
        // Subscribe to lidar
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&ObstacleAvoider::scan_callback, this, std::placeholders::_1));
            
        // Publish velocity commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Obstacle avoider started");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto cmd = geometry_msgs::msg::Twist();
        
        // Get center readings (front of robot)
        int center_idx = msg->ranges.size() / 2;
        int range = 10;  // Check ±10 degrees
        
        // Find minimum distance in front
        float min_distance = 999.0;
        for (int i = center_idx - range; i <= center_idx + range; i++) {
            if (i >= 0 && i < msg->ranges.size()) {
                min_distance = std::min(min_distance, msg->ranges[i]);
            }
        }
        
        // Simple control logic
        if (min_distance < 1.5) {
            // Too close - stop and turn
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;  // Turn left
            RCLCPP_WARN(this->get_logger(), "Obstacle at %.2fm - Turning!", min_distance);
        } else if (min_distance < 8.0) {
            // Slow down
            cmd.linear.x = 0.3;
            cmd.angular.z = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Caution: Obstacle at %.2fm - Slowing down", min_distance);
        } else {
            // Clear - full speed
            cmd.linear.x = 1.0;
            cmd.angular.z = 0.0;
        }
        
        cmd_pub_->publish(cmd);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoider>());
    rclcpp::shutdown();
    return 0;
}