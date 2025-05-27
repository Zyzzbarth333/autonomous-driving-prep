#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor() : Node("lidar_processor")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, 
            std::bind(&LidarProcessor::scan_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Lidar processor started");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Find minimum distance (closest obstacle)
        auto min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
        float min_distance = *min_it;
        int min_index = std::distance(msg->ranges.begin(), min_it);
        
        // Convert index to angle
        float min_angle = msg->angle_min + min_index * msg->angle_increment;
        float min_angle_deg = min_angle * 180.0 / M_PI;
        
        // Detect if obstacle is in front (within ±15 degrees)
        if (std::abs(min_angle_deg) < 15.0 && min_distance < 3.0) {
            RCLCPP_WARN(this->get_logger(), 
                "OBSTACLE AHEAD! Distance: %.2fm at %.1f degrees", 
                min_distance, min_angle_deg);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), 
                *this->get_clock(), 1000,  // Log every 1 second
                "Closest object: %.2fm at %.1f degrees", 
                min_distance, min_angle_deg);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}