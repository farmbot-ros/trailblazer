#include "farmbot_trailblazer/farmtrax/field.hpp"
#include "farmbot_trailblazer/farmtrax/mesh.hpp"
#include "farmbot_trailblazer/farmtrax/plan.hpp"
#include "farmbot_trailblazer/farmtrax/route.hpp"
#include "farmbot_trailblazer/farmtrax/swath.hpp"
#include "farmbot_trailblazer/utils/geojson.hpp"
#include <chrono>
#include <geometry_msgs/msg/detail/polygon__struct.hpp>
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

#include "farmbot_interfaces/msg/polygon_array.hpp"
#include "farmbot_interfaces/msg/segment.hpp"
#include "farmbot_interfaces/msg/segments.hpp"
#include "farmbot_interfaces/msg/swath.hpp"
#include "farmbot_interfaces/msg/swaths.hpp"
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

    geometry_msgs::msg::PolygonStamped outer_polygon_;
    farmbot_interfaces::msg::PolygonArray headlands_;

    rclcpp::TimerBase::SharedPtr planner_timer_;
    rclcpp::Client<farmbot_interfaces::srv::GetTheField>::SharedPtr get_the_field_client_;

    farmbot_interfaces::msg::Swaths swaths_msg_;
    rclcpp::Publisher<farmbot_interfaces::msg::Swaths>::SharedPtr swaths_publisher_;
    rclcpp::Publisher<farmbot_interfaces::msg::PolygonArray>::SharedPtr headlands_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr border;

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
        get_the_field_client_ = this->create_client<farmbot_interfaces::srv::GetTheField>("pln/get_field");

        // Timers
        planner_timer_ = this->create_wall_timer(1s, std::bind(&FieldProcessorNode::planner_timer_cb, this));

        // Swaths publisher
        border = this->create_publisher<geometry_msgs::msg::PolygonStamped>("pln/border", 10);
        swaths_publisher_ = this->create_publisher<farmbot_interfaces::msg::Swaths>("pln/swaths", 10);
        headlands_publisher_ = this->create_publisher<farmbot_interfaces::msg::PolygonArray>("pln/headlands", 10);

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
        border->publish(outer_polygon_);
        // inner_polygon_publisher_->publish(inner_polygon_);
        swaths_publisher_->publish(swaths_msg_);
        headlands_publisher_->publish(headlands_);
    }

    void gen_swaths() {
        std::vector<std::pair<double, double>> points = get_field();
        if (points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get the field");
            return;
        }

        field_.gen_field(points);
        outer_polygon_ = vec_polygon(field_.get_border_points());
        RCLCPP_INFO(this->get_logger(), "Field generated: %lu", field_.get_border_points().size());

        swaths_.gen_swaths(field_, vehicle_coverage_, path_angle_, alternate_freq_);

        swaths_.reverse_swaths();
        RCLCPP_INFO(this->get_logger(), "Swaths generated: %lu", swaths_.get_swaths().size());
        headlands_ = vec_polygon_array(swaths_.get_heardlands());
        plan_.plan_out(swaths_.get_swaths(), alternate_freq_, false);
        RCLCPP_INFO(this->get_logger(), "Plan generated for %i robots", alternate_freq_);

        for (unsigned long i = 0; i < plan_.get_swaths_vec().size(); i++) {
            auto temp_swath_msg = gen_swath_msg(plan_.get_swaths_vec()[i], "robot" + std::to_string(i));
            swaths_msg_.swaths.insert(swaths_msg_.swaths.end(), temp_swath_msg.swaths.begin(),
                                      temp_swath_msg.swaths.end());
        }

        std::vector<farmtrax::Swath> flat_swaths;
        for (const auto &swath : plan_.get_swaths_vec()) {
            flat_swaths.insert(flat_swaths.end(), swath.begin(), swath.end());
        }
    }

    farmbot_interfaces::msg::Swaths gen_swath_msg(const std::vector<farmtrax::Swath> &swaths, std::string robot) {
        farmbot_interfaces::msg::Swaths swaths_msg;
        for (const auto &swath : swaths) {
            farmbot_interfaces::msg::Swath swath_msg;
            geometry_msgs::msg::Polygon polygon;
            for (const auto &point : swath.swath) {
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
        for (const auto &point : navpts) {
            points.emplace_back(std::make_pair(point.x, point.y));
        }
        return points;
    }

    geometry_msgs::msg::PolygonStamped vec_polygon(const std::vector<std::pair<double, double>> &points) {
        geometry_msgs::msg::PolygonStamped polygon;
        polygon.header.frame_id = namespace_ + "/map";
        polygon.header.stamp = rclcpp::Clock().now();
        for (const auto &point : points) {
            geometry_msgs::msg::Point32 p;
            p.x = point.first;
            p.y = point.second;
            polygon.polygon.points.push_back(p);
        }
        return polygon;
    }

    farmbot_interfaces::msg::PolygonArray vec_polygon_array(const std::vector<farmtrax::Polygon> &polygons) {
        farmbot_interfaces::msg::PolygonArray polygon_array;
        for (const auto &polygon : polygons) {
            geometry_msgs::msg::PolygonStamped polygon_stamped;
            polygon_stamped.header.frame_id = namespace_ + "/map";
            polygon_stamped.header.stamp = rclcpp::Clock().now();
            for (const auto &point : polygon.outer()) {
                geometry_msgs::msg::Point32 p;
                p.x = point.x();
                p.y = point.y();
                polygon_stamped.polygon.points.push_back(p);
            }
            polygon_array.polygons.push_back(polygon_stamped);
        }
        return polygon_array;
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
        RCLCPP_ERROR(node->get_logger(), "Could not spin executor: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
