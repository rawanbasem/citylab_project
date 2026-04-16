#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "robot_patrol/action/go_to_pose.hpp"

using namespace std::chrono_literals;

class GoToPose : public rclcpp::Node {
public:
    using GoToPoseMsg = robot_patrol::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseMsg>;

    explicit GoToPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("go_to_pose_node", options) {
        
        using namespace std::placeholders;
        // Action Server
        this->action_server_ = rclcpp_action::create_server<GoToPoseMsg>(
            this,
            "/go_to_pose",
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Action Server Ready");
    }

private:
    rclcpp_action::Server<GoToPoseMsg>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

    geometry_msgs::msg::Pose2D current_pos_;
    
    std::shared_ptr<GoalHandleGoToPose> active_goal_handle_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPoseMsg::Goal> goal) 
        {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Action Called. Target: X=%.2f, Y=%.2f, Theta=%.2f", 
                    goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        // If a goal is already running, abort it 
        if (active_goal_handle_ && active_goal_handle_->is_active()) {
            RCLCPP_INFO(this->get_logger(), "New goal received. Aborting current goal.");
            auto result = std::make_shared<GoToPoseMsg::Result>();
            result->status = false;
            active_goal_handle_->abort(result);
        }

        active_goal_handle_ = goal_handle;
        std::thread(&GoToPose::execute, this, goal_handle).detach();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pos_.theta = yaw; // Current yaw in radians
    }

    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        
        rclcpp::Rate loop_rate(10); // 10Hz control loop
        auto feedback = std::make_shared<GoToPoseMsg::Feedback>();
        auto result = std::make_shared<GoToPoseMsg::Result>();
        auto cmd_vel = geometry_msgs::msg::Twist();

        auto goal = goal_handle->get_goal();
        double desired_x = goal->goal_pos.x;
        double desired_y = goal->goal_pos.y;
        double desired_theta_rad = goal->goal_pos.theta * (M_PI / 180.0); // Convert degrees to radians

        int feedback_counter = 0;
        double distance = 1.0;

        while (rclcpp::ok() && distance > 0.05) {
            // Check for cancellation
            if (!goal_handle->is_active()) return;
            if (goal_handle->is_canceling()) {
                result->status = false;
                goal_handle->canceled(result);
                return;
            }

            double x_err = desired_x - current_pos_.x;
            double y_err = desired_y - current_pos_.y;
            distance = std::sqrt(std::pow(x_err, 2) + std::pow(y_err, 2));

            double target_yaw = std::atan2(y_err, x_err);
            double yaw_err = target_yaw - current_pos_.theta;
            yaw_err = std::fmod(yaw_err + M_PI, 2 * M_PI) - M_PI; // Normalize

            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0.6 * yaw_err; 
            vel_publisher_->publish(cmd_vel);

            if (feedback_counter % 10 == 0) {
                feedback->current_pos = current_pos_;
                goal_handle->publish_feedback(feedback);
            }
            feedback_counter++;
            loop_rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Position reached. Rotating to final orientation...");
        double final_yaw_err = 1.0;
        while (rclcpp::ok() && std::abs(final_yaw_err) > 0.05) {
            if (!goal_handle->is_active()) return;
            if (goal_handle->is_canceling()) {
                result->status = false;
                goal_handle->canceled(result);
                return;
            }

            final_yaw_err = desired_theta_rad - current_pos_.theta;
            final_yaw_err = std::fmod(final_yaw_err + M_PI, 2 * M_PI) - M_PI;

            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.6 * final_yaw_err;
            vel_publisher_->publish(cmd_vel);

            if (feedback_counter % 10 == 0) {
                feedback->current_pos = current_pos_;
                goal_handle->publish_feedback(feedback);
            }
            feedback_counter++;
            loop_rate.sleep();
        }

        if (rclcpp::ok() && goal_handle->is_active()) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            vel_publisher_->publish(cmd_vel);

            result->status = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Action Completed");
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}