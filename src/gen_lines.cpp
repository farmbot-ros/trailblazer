#include <geometry_msgs/msg/detail/polygon__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/logging.hpp>
#include <spdlog/spdlog.h>
#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include "farmbot_planner/utils/geojson.hpp"
#include "farmbot_planner/farmtrax/field.hpp"
#include "farmbot_planner/farmtrax/swath.hpp"
#include "farmbot_planner/farmtrax/plan.hpp"
#include "farmbot_planner/farmtrax/mesh.hpp"
#include "farmbot_planner/farmtrax/route.hpp"
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "farmbot_interfaces/srv/get_the_field.hpp"
#include "farmbot_interfaces/msg/segment.hpp"
#include "farmbot_interfaces/msg/segments.hpp"
#include "farmbot_interfaces/msg/swath.hpp"
#include "farmbot_interfaces/msg/swaths.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace echo = spdlog;
using namespace std::chrono_literals;
using namespace std::placeholders;

class FieldProcessorNode : public rclcpp::Node {
private:
    double vehicle_width_;
    double vehicle_coverage_;
    int alternate_freq_;
    double path_angle_;

    bool planner_initialized_ = false;

    std::string namespace_;

    farmtrax::Field field_;
    farmtrax::Swaths swaths_;
    farmtrax::Plan plan_;

    geometry_msgs::msg::PolygonStamped outer_polygon_;
    geometry_msgs::msg::PolygonStamped inner_polygon_;
    visualization_msgs::msg::MarkerArray field_arrows_;

    rclcpp::TimerBase::SharedPtr planner_timer_;
    rclcpp::Client<farmbot_interfaces::srv::GetTheField>::SharedPtr get_the_field_client_;

    farmbot_interfaces::msg::Swaths swaths_msg_;
    rclcpp::Publisher<farmbot_interfaces::msg::Swaths>::SharedPtr swaths_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr field_arrows_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr outer_polygon_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr inner_polygon_publisher_;

public:
    FieldProcessorNode() : Node("gen_lines",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {
        vehicle_width_ = this->get_parameter_or<double>("vehicle_width", 2.0);
        vehicle_coverage_ = this->get_parameter_or<double>("vehicle_coverage", 3.0);
        alternate_freq_ = this->get_parameter_or<int>("alternate_freq", 1);
        path_angle_ = this->get_parameter_or<double>("path_angle", 90);

        // Create the service clients
        get_the_field_client_ = this->create_client<farmbot_interfaces::srv::GetTheField>("pln/get_field");

        // Create publishers
        inner_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("pln/inner_field", 10);
        outer_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("pln/outer_field", 10);
        field_arrows_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pln/arrow_swath", 10);

        // Timers
        planner_timer_ = this->create_wall_timer(1s, std::bind(&FieldProcessorNode::planner_timer_cb, this));

        // Swaths publisher
        swaths_publisher_ = this->create_publisher<farmbot_interfaces::msg::Swaths>("/pln/swaths", 10);

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
        std::thread([this] {
            gen_swaths();
        }).detach();
    }

private:
    void planner_timer_cb() {
        if (!planner_initialized_) { return; }
        outer_polygon_publisher_->publish(outer_polygon_);
        inner_polygon_publisher_->publish(inner_polygon_);
        field_arrows_pub_->publish(field_arrows_);
        swaths_publisher_->publish(swaths_msg_);
    }

    void gen_swaths() {
        std::vector<std::pair<double, double>> points = get_field();
        if (points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get the field");
            return;
        }

        field_.gen_field(points);
        farmtrax::Field hl = field_.get_buffered(vehicle_width_ * 2.0, farmtrax::BufferType::SHRINK);
        outer_polygon_ = vector2Polygon(field_.get_border_points());
        inner_polygon_ = vector2Polygon(hl.get_border_points());
        RCLCPP_INFO(this->get_logger(), "Field generated: %lu", field_.get_border_points().size());

        swaths_.gen_swaths(field_, hl, vehicle_coverage_, path_angle_, vehicle_width_);
        swaths_.reverse_swaths();
        RCLCPP_INFO(this->get_logger(), "Swaths generated: %lu", swaths_.get_swaths().size());

        plan_.plan_out(swaths_.get_swaths(), alternate_freq_, false);
        RCLCPP_INFO(this->get_logger(), "Plan generated for %i robots", alternate_freq_);


        for (unsigned long i = 0; i < plan_.get_swaths_vec().size(); i++) {
            auto temp_swath_msg = swath2SwathMsg(plan_.get_swaths_vec()[i], "robot"+std::to_string(i));
            swaths_msg_.swaths.insert(swaths_msg_.swaths.end(), temp_swath_msg.swaths.begin(), temp_swath_msg.swaths.end());
        }

        std::vector<farmtrax::Swath> flat_swaths;
        for (const auto& swath : plan_.get_swaths_vec()) {
            flat_swaths.insert(flat_swaths.end(), swath.begin(), swath.end());
        }
        field_arrows_ = vector2ArrowsColor(flat_swaths);
        planner_initialized_ = true;
    }

    farmbot_interfaces::msg::Swaths swath2SwathMsg(const std::vector<farmtrax::Swath>& swaths, std::string robot) {
        farmbot_interfaces::msg::Swaths swaths_msg;
        for (const auto& swath : swaths) {
            farmbot_interfaces::msg::Swath swath_msg;
            geometry_msgs::msg::Polygon polygon;
            for (const auto& point : swath.swath) {
                geometry_msgs::msg::Point32 p;
                p.x = point.x();
                p.y = point.y();
                polygon.points.push_back(p);
            }
            swath_msg.line = polygon;
            swath_msg.robot.data = robot;
            swath_msg.length.data = swath.length;
            swath_msg.uuid.data = swath.uuid;
            swath_msg.type.data = static_cast<uint8_t>(swath.type);
            swaths_msg.swaths.push_back(swath_msg);
        }
        return swaths_msg;
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
        for (const auto& point : navpts) {
            points.emplace_back(std::make_pair(point.x, point.y));
        }
        return points;
    }

    geometry_msgs::msg::PolygonStamped vector2Polygon(const std::vector<std::pair<double, double>>& points) {
        geometry_msgs::msg::PolygonStamped polygon;
        polygon.header.frame_id = namespace_ + "/map";
        polygon.header.stamp = rclcpp::Clock().now();
        for (const auto& point : points) {
            geometry_msgs::msg::Point32 p;
            p.x = point.first;
            p.y = point.second;
            polygon.polygon.points.push_back(p);
        }
        return polygon;
    }

    visualization_msgs::msg::MarkerArray vector2ArrowsColor(const std::vector<farmtrax::Swath>& swaths) {
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;
        int num_swaths = swaths.size();
        for (const auto& swath : swaths) {
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = namespace_ + "/map";
            arrow.header.stamp = rclcpp::Clock().now();
            arrow.ns = "swath_arrows";
            arrow.id = id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.scale.x = 0.2;  // Shaft diameter
            arrow.scale.y = 1;    // Head diameter
            arrow.scale.z = 2.0;  // Head length

            float ratio = static_cast<float>(id) / num_swaths;
            if (ratio < 0.5) {
                arrow.color.r = 1.0f - 2.0f * ratio;
                arrow.color.g = 0.5f + 1.0f * ratio;
                arrow.color.b = 2.0f * ratio;
            } else {
                arrow.color.r = 2.0f * (ratio - 0.5f);
                arrow.color.g = 1.0f - 2.0f * (ratio - 0.5f);
                arrow.color.b = 1.0f;
            }
            arrow.color.a = 1.0f;
            arrow.lifetime = rclcpp::Duration::from_seconds(0);
            geometry_msgs::msg::Point p1;
            p1.x = swath.swath[0].x();
            p1.y = swath.swath[0].y();
            geometry_msgs::msg::Point p2;
            p2.x = swath.swath[1].x();
            p2.y = swath.swath[1].y();
            arrow.points.push_back(p1);
            arrow.points.push_back(p2);
            markers.markers.push_back(arrow);
        }
        return markers;
    }
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
        RCLCPP_ERROR(node->get_logger(), e.what());
    }
    rclcpp::shutdown();
    return 0;
}
