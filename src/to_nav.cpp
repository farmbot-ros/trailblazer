#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "farmbot_interfaces/action/waypoints.hpp"
#include "farmbot_interfaces/action/control.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include <farmbot_interfaces/action/detail/waypoints__struct.hpp>
#include "farmbot_interfaces/msg/waypoint.hpp"
#include "farmbot_interfaces/msg/segment.hpp"
#include "farmbot_interfaces/msg/segments.hpp"
#include "farmbot_interfaces/srv/go_to_field.hpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <iostream>
#include <rclcpp_action/server_goal_handle.hpp>
#include <thread>
#include <mutex>
#include <string>

using namespace std::chrono_literals;
using namespace std::placeholders;
using Trigger = farmbot_interfaces::srv::Trigger;


class Navigator : public rclcpp::Node {
private:
    std::string namespace_;
    farmbot_interfaces::msg::Waypoints mission_;
    bool gotofield_;

    // Waypoints
    farmbot_interfaces::msg::Waypoints path_nav;

    // Segments subscriber
    rclcpp::Subscription<farmbot_interfaces::msg::Segments>::SharedPtr segments_sub_;
    farmbot_interfaces::msg::Segments segments_;

    // Action client
    rclcpp_action::Client<farmbot_interfaces::action::Waypoints>::SharedPtr control_client_;
    rclcpp_action::ClientGoalHandle<farmbot_interfaces::action::Waypoints>::SharedPtr waypoints_goal_handle_;

    // Service client
    rclcpp::Client<farmbot_interfaces::srv::GoToField>::SharedPtr goto_field_client_;
    farmbot_interfaces::msg::Segments goto_field_segments_;

    // Diagnostic
    diagnostic_updater::Updater updater_;
    diagnostic_msgs::msg::DiagnosticStatus status;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

public:
    Navigator() : Node("to_nav",
        rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)
    ), updater_(this) {
        gotofield_ = this->get_parameter_or<bool>("gotofield", false);

        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Not initialized";

        // Action client
        control_client_ = rclcpp_action::create_client<farmbot_interfaces::action::Waypoints>(this, "nav/mission");
        // Service client
        goto_field_client_ = this->create_client<farmbot_interfaces::srv::GoToField>("pln/get_route");
        // Subscriber
        segments_sub_ = this->create_subscription<farmbot_interfaces::msg::Segments>("pln/segments", 10, std::bind(&Navigator::segments_callback, this, _1));

        namespace_ = this->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1); // Remove leading slash
        }

        // Diagnostic Updater
        updater_.setHardwareID(static_cast<std::string>(this->get_namespace()) + "/pla");
        updater_.add("Planning Status", this, &Navigator::check_system);
        diagnostic_timer_ = this->create_wall_timer(1s, std::bind(&Navigator::diagnostic_callback, this));
    }

private:
    void diagnostic_callback() {
        updater_.force_update();
    }

    void check_system(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        stat.summary(status.level, status.message);
    }

    void segments_callback(const farmbot_interfaces::msg::Segments::SharedPtr msg) {
        segments_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received path");

        if (gotofield_) {
            RCLCPP_INFO(this->get_logger(), "Received path for go to field");
            go_to_field(segments_.segments[0].origin.pose);
        }
        send_waypoints_goal();
        // Send action goal
        this->segments_sub_.reset();
    }

    void send_waypoints_goal() {
        nav_msgs::msg::Path path;
        geometry_msgs::msg::PoseStamped pose_stamped;
        for (int i = 0; i < segments_.segments.size(); i += 1) {
            pose_stamped.pose = segments_.segments[i].origin.pose;
            path.poses.push_back(pose_stamped);
            pose_stamped.pose = segments_.segments[i].destination.pose;
            path.poses.push_back(pose_stamped);
        }
        while (!control_client_->wait_for_action_server(1s) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server to be ready...");
        }
        // Create the goal message
        auto goal_msg = farmbot_interfaces::action::Waypoints::Goal();
        // Convert nav_msgs::msg::Path to farmbot_interfaces::msg::Waypoints
        mission_.header = path.header;
        for (const auto& pose_stamped : path.poses) {
            farmbot_interfaces::msg::Waypoint waypoint;
            waypoint.header = pose_stamped.header;
            waypoint.pose = pose_stamped.pose;
            waypoint.uuid.data = "";
            mission_.poses.push_back(waypoint);
        }
        goal_msg.mission = mission_;
        // Send the goal
        auto send_goal_options = rclcpp_action::Client<farmbot_interfaces::action::Waypoints>::SendGoalOptions();
        // Define callbacks
        send_goal_options.goal_response_callback = std::bind(&Navigator::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&Navigator::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&Navigator::result_callback, this, _1);

        auto future_goal_handle = control_client_->async_send_goal(goal_msg, send_goal_options);
    }


    void go_to_field(geometry_msgs::msg::Pose point) {
        auto request = std::make_shared<farmbot_interfaces::srv::GoToField::Request>();
        request->point = point;
        while (!goto_field_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = goto_field_client_->async_send_request(request);
        while (rclcpp::ok() && result.wait_for(1s) == std::future_status::timeout) {
            RCLCPP_INFO(this->get_logger(), "Waiting for response for go to field...");
        }
        auto points =  result.get()->points;
        for (int i = 0; i < points.size(); i += 1) {
            farmbot_interfaces::msg::Waypoint waypoint;
            waypoint.pose.position = points[i];
            waypoint.uuid.data = "";
            mission_.poses.push_back(waypoint);
        }
    }

    void goal_response_callback(rclcpp_action::ClientGoalHandle<farmbot_interfaces::action::Waypoints>::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Goal rejected by server";
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            waypoints_goal_handle_ = goal_handle;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Goal accepted by server";
        }
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<farmbot_interfaces::action::Waypoints>::SharedPtr,
        const std::shared_ptr<const farmbot_interfaces::action::Waypoints::Feedback> feedback) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received feedback");
        // Optionally update diagnostic status or other variables
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<farmbot_interfaces::action::Waypoints>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                status.message = "Goal succeeded";
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Goal was aborted");
                status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = "Goal aborted";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = "Goal canceled";
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Unknown result code");
                status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = "Unknown result code";
                break;
        }
        // After the goal is complete, re-enable the subscriber to receive new paths
        // if (!this->path_sub) {
            // this->path_sub = this->create_subscription<nav_msgs::msg::Path>("path", 10, std::bind(&Navigator::path_callback, this, _1));
        // }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Navigator>();
    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), e.what());
    }
    rclcpp::shutdown();
    return 0;
}
