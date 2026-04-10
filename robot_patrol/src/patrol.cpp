#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("robot_patrol_node") {
        
        // Subscribing to the simulation's laser scan topic
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

        // Publishing to the simulation's velocity topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

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
        
        int size = msg->ranges.size();
        int quarter_index = size / 4;
        bool obstacle_in_front = false;
        int cone_size = 10; // Check 10 rays on each side of the front

        // Check the front-left
        for (int i = 0; i <= cone_size; i++) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < 0.55) {
                obstacle_in_front = true;
            }
        }
        //Check the front-right
        for (int i = size - cone_size; i < size; i++) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < 0.55) {
                obstacle_in_front = true;
            }
        }

        if (obstacle_in_front) {
            float max_dist = -1.0;
            int max_idx = 0; 

            // Search left side (0 to pi/2)
            for (int i = 0; i <= quarter_index; i++) {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > max_dist) {
                    max_dist = msg->ranges[i];
                    max_idx = i; 
                }    
            }
            
            // Search right side (3pi/2 to 2pi)
            for (int i = size - quarter_index; i < size; i++) {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > max_dist) {
                    max_dist = msg->ranges[i];
                    max_idx = i; 
                }    
            }
            
            double raw_angle = msg->angle_min + (max_idx * msg->angle_increment);
            
            // normalize angles
            if (raw_angle > M_PI) {
                raw_angle -= 2 * M_PI;
            }
            
            this->direction_ = raw_angle;
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