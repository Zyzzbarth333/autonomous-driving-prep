#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

class IntegratedParkingController : public rclcpp::Node
{
public:
    IntegratedParkingController() : Node("integrated_parking_controller"),
                                   state_("SEARCHING")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&IntegratedParkingController::scan_callback, this, std::placeholders::_1));
            
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        state_pub_ = this->create_publisher<std_msgs::msg::String>("parking_state", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&IntegratedParkingController::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Integrated parking controller started");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;
        
        if (state_ == "SEARCHING") {
            // Look for parking space (gap in obstacles)
            bool space_found = detect_parking_space(msg);
            if (space_found) {
                state_ = "ALIGNING";
                RCLCPP_INFO(this->get_logger(), "Parking space detected! Starting alignment...");
            }
        }
    }
    
    bool detect_parking_space(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Look for a sufficiently large gap
        int consecutive_far = 0;
        int required_gap = 20;  // Adjust based on angle increment
        
        for (size_t i = msg->ranges.size()/3; i < 2*msg->ranges.size()/3; i++) {
            if (msg->ranges[i] > 8.0) {
                consecutive_far++;
                if (consecutive_far >= required_gap) {
                    space_center_idx_ = i - required_gap/2;
                    return true;
                }
            } else {
                consecutive_far = 0;
            }
        }
        return false;
    }
    
    void control_loop()
    {
        auto cmd = geometry_msgs::msg::Twist();
        
        if (state_ == "SEARCHING") {
            // Slowly move forward while searching
            cmd.linear.x = 0.3;
            cmd.angular.z = 0.0;
        } 
        else if (state_ == "ALIGNING") {
            // Stop and prepare for parking
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            
            align_counter_++;
            if (align_counter_ > 20) {  // 2 seconds
                state_ = "PARKING";
                RCLCPP_INFO(this->get_logger(), "Starting parking maneuver...");
            }
        }
        else if (state_ == "PARKING") {
            // Simple reverse parking
            parking_counter_++;
            
            if (parking_counter_ < 30) {
                // Reverse with slight turn
                cmd.linear.x = -0.2;
                cmd.angular.z = 0.3;
            } else if (parking_counter_ < 50) {
                // Straighten out
                cmd.linear.x = -0.1;
                cmd.angular.z = -0.2;
            } else {
                // Stop - parking complete
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                state_ = "COMPLETE";
                RCLCPP_INFO(this->get_logger(), "Parking complete!");
            }
        }
        
        cmd_pub_->publish(cmd);
        
        // Publish state
        auto state_msg = std_msgs::msg::String();
        state_msg.data = state_;
        state_pub_->publish(state_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    std::string state_;
    int space_center_idx_ = 0;
    int align_counter_ = 0;
    int parking_counter_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntegratedParkingController>());
    rclcpp::shutdown();
    return 0;
}