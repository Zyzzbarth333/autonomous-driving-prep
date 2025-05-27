#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class LidarSimulator : public rclcpp::Node
{
public:
    LidarSimulator() : Node("lidar_simulator"), gen_(rd_()), dist_(5.0, 0.1)
    {
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        
        // Create timer that fires at 10Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&LidarSimulator::publish_scan, this));
            
        RCLCPP_INFO(this->get_logger(), "Lidar simulator started");
    }

private:
    void publish_scan()
    {
        auto message = sensor_msgs::msg::LaserScan();
        
        // Header
        message.header.stamp = this->now();
        message.header.frame_id = "laser_frame";
        
        // Lidar parameters
        message.angle_min = -1.57;  // -90 degrees
        message.angle_max = 1.57;   // 90 degrees
        message.angle_increment = 0.0174533;  // 1 degree
        message.range_min = 0.5;
        message.range_max = 30.0;
        
        // Generate simulated ranges
        int num_readings = (message.angle_max - message.angle_min) / message.angle_increment;
        message.ranges.resize(num_readings);
        
        for (int i = 0; i < num_readings; ++i) {
            // Simulate a wall at 5m with noise
            message.ranges[i] = 5.0 + dist_(gen_);
            
            // Simulate an obstacle at 45 degrees
            if (i > num_readings/2 - 5 && i < num_readings/2 + 5) {
                message.ranges[i] = 2.0 + dist_(gen_);
            }
        }
        
        publisher_->publish(message);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimulator>());
    rclcpp::shutdown();
    return 0;
}