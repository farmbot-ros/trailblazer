#include "farmbot_trailblazer/farmtrax/field.hpp"
#include "farmbot_trailblazer/farmtrax/mesh.hpp"
#include "farmbot_trailblazer/farmtrax/plan.hpp"
#include "farmbot_trailblazer/farmtrax/route.hpp"
#include "farmbot_trailblazer/farmtrax/swath.hpp"
#include "farmbot_trailblazer/utils/geojson.hpp"
#include <chrono>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/logging.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "farmbot_interfaces/msg/line.hpp"
#include "farmbot_interfaces/msg/lines.hpp"
#include "farmbot_interfaces/srv/get_the_field.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// namespace echo = spdlog;
using namespace std::chrono_literals;
using namespace std::placeholders;

class FieldProcessorNode : public rclcpp::Node {
  private:
    double vehicle_coverage_;
    int alternate_freq_;
    double path_angle_;

    bool planner_initialized_ = false;

    std::string namespace_;

    farmtrax::Field field_;
    farmtrax::Swaths swaths_;
    farmtrax::Plan plan_;

    farmbot_interfaces::msg::Lines border_msg_;
    farmbot_interfaces::msg::Lines headlands_msg_;
    farmbot_interfaces::msg::Lines swaths_msg_;

    rclcpp::TimerBase::SharedPtr planner_timer_;
    rclcpp::Client<farmbot_interfaces::srv::GetTheField>::SharedPtr get_the_field_client_;

    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr swaths_publisher_;
    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr headlands_publisher_;
    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr border_publisher_;

  public:
    FieldProcessorNode()
        : Node("gen_lines",
               rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
                   true)) {
        vehicle_coverage_ = this->get_parameter_or<double>("vehicle_coverage", 3.0);
        // Alternate frequency is the number of robots in the swath
        alternate_freq_ = this->get_parameter_or<int>("alternate_freq", 1);
        path_angle_ = this->get_parameter_or<double>("path_angle", 90);
        RCLCPP_INFO(this->get_logger(), "swath frequency: %i", alternate_freq_);

        // Create the service clients
        get_the_field_client_ = this->create_client<farmbot_interfaces::srv::GetTheField>("/field/get_field");

        // Timers
        planner_timer_ = this->create_wall_timer(1s, std::bind(&FieldProcessorNode::planner_timer_cb, this));

        // Line publisher
        border_publisher_ = this->create_publisher<farmbot_interfaces::msg::Lines>("/field/border", 10);
        swaths_publisher_ = this->create_publisher<farmbot_interfaces::msg::Lines>("/field/swaths", 10);
        headlands_publisher_ = this->create_publisher<farmbot_interfaces::msg::Lines>("/field/headlands", 10);

        // Namespace
        namespace_ = this->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }
    }

    void init() {
        swaths_.pass_node(this->shared_from_this());
        plan_.pass_node(this->shared_from_this());
        field_.pass_node(this->shared_from_this());
        std::thread([this] { gen_swaths(); }).detach();
    }

  private:
    void planner_timer_cb() {
        if (!planner_initialized_) {
            return;
        }
        border_publisher_->publish(border_msg_);
        swaths_publisher_->publish(swaths_msg_);
        headlands_publisher_->publish(headlands_msg_);
    }

    void gen_swaths() {
        std::vector<std::pair<double, double>> points = get_field();
        if (points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get the field");
            return;
        }

        field_.gen_field(points);
        // border_msg_ = vec_polygon(field_.get_border_points());
        RCLCPP_INFO(this->get_logger(), "Field generated: %lu", field_.get_border_points().size());

        swaths_.gen_swaths(field_, vehicle_coverage_, path_angle_, alternate_freq_);

        swaths_.reverse_swaths();
        RCLCPP_INFO(this->get_logger(), "Lines generated: %lu", swaths_.get_swaths().size());
        // headlands_msg_ = vec_polygon_array(swaths_.get_heardlands());
        plan_.plan_out(swaths_.get_swaths(), alternate_freq_, false);
        RCLCPP_INFO(this->get_logger(), "Plan generated for %i robots", alternate_freq_);

        for (unsigned long i = 0; i < plan_.get_swaths_vec().size(); i++) {
            auto temp_swath_msg = swaths_to_msg(plan_.get_swaths_vec()[i], "robot" + std::to_string(i));
            swaths_msg_.lines.insert(swaths_msg_.lines.end(), temp_swath_msg.lines.begin(), temp_swath_msg.lines.end());
        }
        //
        // std::vector<farmtrax::Swath> flat_swaths;
        // for (const auto &swath : plan_.get_swaths_vec()) {
        //     flat_swaths.insert(flat_swaths.end(), swath.begin(), swath.end());
        // }
        // planner_initialized_ = true;
    }

    std::vector<std::pair<double, double>> get_field(std::string geojson_file_path = "") {
        std::vector<std::pair<double, double>> points;
        auto request = std::make_shared<farmbot_interfaces::srv::GetTheField::Request>();
        request->geojson_file = geojson_file_path;
        while (!get_the_field_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return {};
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        auto result = get_the_field_client_->async_send_request(request);
        auto navpts = result.get()->points;
        for (const auto &point : navpts) {
            points.emplace_back(std::make_pair(point.x, point.y));
        }
        return points;
    }

    farmbot_interfaces::msg::Lines swaths_to_msg(const std::vector<farmtrax::Swath> &swaths, std::string robot) {
        farmbot_interfaces::msg::Lines swaths_msg;
        for (const auto &swath : swaths) {
            farmbot_interfaces::msg::Line swath_msg;
            std::vector<geometry_msgs::msg::Point> loc_line;
            for (const auto &point : swath.swath) {
                geometry_msgs::msg::Point p;
                p.x = point.x();
                p.y = point.y();
                loc_line.push_back(p);
            }
            swath_msg.loc_line = loc_line;
            swath_msg.robot = robot;
            swath_msg.length = swath.length;
            swath_msg.uuid = swath.uuid;
            swath_msg.type = static_cast<uint8_t>(swath.type);
            swaths_msg.lines.push_back(swath_msg);
        }
        return swaths_msg;
    }

    //
    // farmbot_interfaces::msg::Line vec_swaths(const std::vector<std::pair<double, double>> &points) {
    //     farmbot_interfaces::msg::Line message;
    //     message.header.frame_id = namespace_ + "/map";
    //     message.header.stamp = rclcpp::Clock().now();
    //     for (const auto &point : points) {
    //         farmbot_interfaces::msg::Swath swath;
    //
    //         p.x = point.first;
    //         p.y = point.second;
    //     }
    //
    // }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<FieldProcessorNode>();
    node->init();
    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Could not spin executor: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
