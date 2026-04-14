#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TestService : public rclcpp::Node {
public:
    TestService() : Node("test_service_node") {
        
        client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");
        
        // Wait for the service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        RCLCPP_INFO(this->get_logger(), "Service Client Ready! Listening to laser...");

        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10, std::bind(&TestService::laserscan_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_; 
    
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg; 
        RCLCPP_INFO(this->get_logger(), "Service Request sent...");

        client_->async_send_request(request, 
            [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->direction.c_str());
            }
        );
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}