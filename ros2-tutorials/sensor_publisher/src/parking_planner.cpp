#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ParkingPlanner : public rclcpp::Node
{
public:
    ParkingPlanner() : Node("parking_planner")
    {
        // Subscribe to lidar for space detection
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&ParkingPlanner::scan_callback, this, std::placeholders::_1));
            
        // Publish parking trajectory
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("parking_path", 10);
        
        RCLCPP_INFO(this->get_logger(), "Parking planner started");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Simple parking space detection
        // Look for a gap of at least 2.5m (typical parking space)
        
        std::vector<std::pair<int, int>> gaps;
        float space_threshold = 8.0;  // Distance to wall
        float min_width = 2.5;  // Minimum parking space width
        
        int gap_start = -1;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] > space_threshold) {
                if (gap_start == -1) gap_start = i;
            } else {
                if (gap_start != -1) {
                    // Calculate gap width
                    float angle_diff = (i - gap_start) * msg->angle_increment;
                    float width = msg->ranges[gap_start] * sin(angle_diff);
                    
                    if (std::abs(width) > min_width) {
                        gaps.push_back({gap_start, i});
                        RCLCPP_INFO(this->get_logger(), 
                            "Found parking space: angles %d to %d", gap_start, i);
                    }
                    gap_start = -1;
                }
            }
        }
        
        // Generate parking path for first suitable space
        if (!gaps.empty()) {
            generate_parking_path(msg, gaps[0]);
        }
    }
    
    void generate_parking_path(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                              const std::pair<int, int>& gap)
    {
        auto path = nav_msgs::msg::Path();
        path.header.stamp = this->now();
        path.header.frame_id = "laser_frame";
        
        // Simple parallel parking trajectory
        // 1. Drive past the space
        // 2. Reverse while turning
        // 3. Straighten out
        
        int center_idx = (gap.first + gap.second) / 2;
        float center_angle = scan->angle_min + center_idx * scan->angle_increment;
        float distance = scan->ranges[center_idx];
        
        // Generate waypoints
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            
            // Simple trajectory - customize based on parking type
            float progress = i / 9.0;
            pose.pose.position.x = progress * distance * cos(center_angle);
            pose.pose.position.y = progress * distance * sin(center_angle);
            pose.pose.position.z = 0.0;
            
            path.poses.push_back(pose);
        }
        
        path_pub_->publish(path);
        RCLCPP_INFO_ONCE(this->get_logger(), 
            "Published parking path with %zu waypoints", path.poses.size());
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParkingPlanner>());
    rclcpp::shutdown();
    return 0;
}