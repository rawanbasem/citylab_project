#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("robot_patrol_node") {
        
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10, std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);

        // 10 Hz control loop
        timer_ = this->create_wall_timer(100ms, std::bind(&Patrol::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Robot Patrol Node started...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double direction_ = 0.0;

    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        
        int middle_index = msg->ranges.size() / 2;
        float forward_dist = msg->ranges[middle_index];

        if (std::isfinite(forward_dist) && forward_dist < 0.55) {
            
            int first_index = msg->ranges.size() / 4;
            int last_index = 3 * (msg->ranges.size()) / 4;
            float max_dist = -1.0;
            int max_idx = middle_index; 

            // Find safest direction 
            for (int i = first_index; i <= last_index; i++) {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > max_dist) {
                    max_dist = msg->ranges[i];
                    max_idx = i; 
                }    
            }
            this->direction_ = msg->angle_min + (max_idx * msg->angle_increment);
        } 
        else {
            // Path is clear 
            this->direction_ = 0.0;
        }
    }

    void timer_callback() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.1;               
        msg.angular.z = direction_ / 2.0; 
        publisher_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}