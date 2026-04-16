#include "robot_patrol/srv/get_direction.hpp"
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class PatrolWithService : public rclcpp::Node {
public:
  PatrolWithService() : Node("patrol_with_service_node") {

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&PatrolWithService::laserscan_callback, this,
                  std::placeholders::_1));

    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service");

    timer_ = this->create_wall_timer(
        100ms, std::bind(&PatrolWithService::timer_callback, this));

    // Initialize default movement
    cmd_vel_.linear.x = 0.1;
    cmd_vel_.angular.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Patrol With Service Node Started!");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist cmd_vel_;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    int middle_index = msg->ranges.size() / 2;
    float forward_dist = msg->ranges[middle_index];

    // If obstacle is closer than 55cm:
    if (std::isfinite(forward_dist) && forward_dist < 0.55) {

      auto request =
          std::make_shared<robot_patrol::srv::GetDirection::Request>();
      request->laser_data = *msg;

      // Sending request asynchronously
      client_->async_send_request(
          request,
          [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
            auto response = future.get();

            if (response->direction == "forward") {
              cmd_vel_.linear.x = 0.1;
              cmd_vel_.angular.z = 0.0;
            } else if (response->direction == "left") {
              cmd_vel_.linear.x = 0.1;
              cmd_vel_.angular.z = 0.5;
            } else if (response->direction == "right") {
              cmd_vel_.linear.x = 0.1;
              cmd_vel_.angular.z = -0.5;
            }
          });
    } else {
      // Path is clear
      cmd_vel_.linear.x = 0.1;
      cmd_vel_.angular.z = 0.0;
    }
  }

  void timer_callback() {
    publisher_->publish(cmd_vel_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolWithService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}