#include "robot_patrol/srv/get_direction.hpp"
#include <rclcpp/rclcpp.hpp>

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    std::string name_service = "/direction_service";

    service_ = this->create_service<robot_patrol::srv::GetDirection>(
        name_service, std::bind(&DirectionService::service_callback, this,
                                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...", name_service.c_str());
  }

private:
    rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;


    void service_callback(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {

        // Rotate array 180 degrees for real robot
        int size = request->laser_data.ranges.size();
        int half_size = size / 2;
        std::vector<float> rotated_ranges(size);
        
        for (int i = 0; i < size; i++) {
            rotated_ranges[i] = request->laser_data.ranges[(i + half_size) % size];
        }
        request->laser_data.ranges = rotated_ranges;
        
        RCLCPP_INFO(this->get_logger(), "Service Requested");

        double total_dist_sec_right = 0.0;
        double total_dist_sec_front = 0.0;
        double total_dist_sec_left = 0.0;

        int first_index = request->laser_data.ranges.size() / 4;
        int last_index = 3 * (request->laser_data.ranges.size()) / 4;
        int total_rays = last_index - first_index;
        int chunk_size = total_rays / 3;
        int right_end = first_index + chunk_size;
        int front_end = first_index + (2 * chunk_size);

        for (int i = first_index; i <= last_index; i++) {
            //  safety check
            if (std::isfinite(request->laser_data.ranges[i])) {
            
                // Sort into buckets
                if (i < right_end) {
                    total_dist_sec_right += request->laser_data.ranges[i];
                } 
                else if (i < front_end) {
                    total_dist_sec_front += request->laser_data.ranges[i];
                }
                else { //left
                    total_dist_sec_left += request->laser_data.ranges[i];
                }
            }
            // RCLCPP_INFO(this->get_logger(), "Totals -> Right: %f | Front: %f | Left: %f", total_dist_sec_right, total_dist_sec_front, total_dist_sec_left);
        }

        if (total_dist_sec_right > total_dist_sec_front && total_dist_sec_right > total_dist_sec_left) {
            response->direction = "right";
        }
        else if (total_dist_sec_front > total_dist_sec_left && total_dist_sec_front > total_dist_sec_right) {
            response->direction = "forward";
        }
        else if (total_dist_sec_left > total_dist_sec_right && total_dist_sec_left > total_dist_sec_front) {
            response->direction = "left";
        }
        RCLCPP_INFO(this->get_logger(), "Service Completed");
    }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}