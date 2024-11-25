#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include "farmbot_planner/utils/geojson.hpp"
#include "farmbot_planner/farmtrax/field.hpp"
#include "farmbot_planner/farmtrax/swath.hpp"
#include "farmbot_planner/farmtrax/mesh.hpp"
#include "farmbot_planner/farmtrax/route.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "farmbot_interfaces/srv/get_the_field.hpp"
#include "farmbot_interfaces/msg/segment.hpp"
#include "farmbot_interfaces/msg/segments.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

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

    farmtrax::Mesh mesh_;
    farmtrax::Route route_;

    visualization_msgs::msg::MarkerArray field_arrows_;
    geometry_msgs::msg::PolygonStamped outer_polygon_;
    geometry_msgs::msg::PolygonStamped inner_polygon_;

    rclcpp::TimerBase::SharedPtr visualization_timer_;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    rclcpp::Client<farmbot_interfaces::srv::GetTheField>::SharedPtr get_the_field_client_;

    rclcpp::TimerBase::SharedPtr on_timer_;
    farmbot_interfaces::msg::Segments segments_;
    rclcpp::Publisher<farmbot_interfaces::msg::Segments>::SharedPtr segment_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr field_arrows_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr outer_polygon_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr inner_polygon_publisher_;

public:
    FieldProcessorNode() : Node("ab_planner",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {
        vehicle_width_ = this->get_parameter_or<double>("vehicle_width", 0.5);
        vehicle_coverage_ = this->get_parameter_or<double>("vehicle_coverage", 3.0);
        alternate_freq_ = this->get_parameter_or<int>("alternate_freq", 1);
        path_angle_ = this->get_parameter_or<double>("path_angle", 90);

        // Create the service clients
        gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>("loc/gps2enu");
        get_the_field_client_ = this->create_client<farmbot_interfaces::srv::GetTheField>("pln/get_field");

        // Create publishers
        inner_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("pln/inner_field", 10);
        outer_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("pln/outer_field", 10);
        field_arrows_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pln/arrow_swath", 10);

        // Segment publisher
        segment_publisher_ = this->create_publisher<farmbot_interfaces::msg::Segments>("pln/segments", 10);
        on_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FieldProcessorNode::on_timer, this));

        // Timers
        visualization_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FieldProcessorNode::on_service_start_timer, this));

        // Namespace
        namespace_ = this->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }

        // Run the main processing in a separate thread
        std::thread([this] { runner(); }).detach();
    }

private:
    void on_service_start_timer() {
        if (!planner_initialized_) { return; }
        outer_polygon_publisher_->publish(outer_polygon_);
        inner_polygon_publisher_->publish(inner_polygon_);
        field_arrows_pub_->publish(field_arrows_);
    }

    void on_timer() {
        if (!planner_initialized_) { return; }
        segment_publisher_->publish(segments_);
    }

    void runner() {
        auto getfield = get_the_field();
        if (getfield.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No points found");
            return;
        }
        auto field_in_enu = nav_to_enu(getfield);
        if (field_in_enu.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No ENU points received.");
            return;
        }
        process_field(field_in_enu);
        planner_initialized_ = true;
    }

    void process_field(std::vector<std::pair<double, double>> points) {
        field_.gen_field(points);
        farmtrax::Field hl = field_.get_buffered(vehicle_width_ * 2.0, farmtrax::BufferType::SHRINK);
        RCLCPP_INFO(this->get_logger(), "Field generated: %lu", field_.get_border_points().size());

        outer_polygon_ = vector2Polygon(field_.get_border_points());
        inner_polygon_ = vector2Polygon(hl.get_border_points());
        swaths_.gen_swaths(field_, hl, vehicle_coverage_, path_angle_, alternate_freq_, vehicle_width_);
        RCLCPP_INFO(this->get_logger(), "Swaths generated: %lu", swaths_.get_swaths().size());

        // mesh_ = farmtrax::Mesh(swaths_);
        mesh_.build_graph(swaths_);

        route_.find_optimal(mesh_, farmtrax::Route::Algorithm::EXHAUSTIVE_SEARCH);
        field_arrows_ = vector2ArrowsColor(route_.get_swaths());
        // segments_ = vector2Segments(swath_path);
        RCLCPP_INFO(this->get_logger(), "Route generated: %lu", route_.get_swaths().size());
    }

    std::vector<std::vector<double>> get_the_field(std::string geojson_file_path = "") {
        std::vector<std::vector<double>> points;
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
        auto navpts = result.get()->field;
        for (const auto& point : navpts) {
            points.emplace_back(std::vector<double>{point.latitude, point.longitude});
        }
        return points;
    }

    std::vector<std::pair<double, double>> nav_to_enu(const std::vector<std::vector<double>>& navpts) {
        std::vector<std::pair<double, double>> points;
        auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();
        for (const auto& point : navpts) {
            sensor_msgs::msg::NavSatFix gps_point;
            gps_point.latitude = point[0];
            gps_point.longitude = point[1];
            gps_point.altitude = 0.0;  // Adjust if altitude data is available
            request->gps.push_back(gps_point);
        }
        while (!gps2enu_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return {};
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        auto result = gps2enu_client_->async_send_request(request);
        auto getres = result.get()->enu;
        for (const auto& point : getres) {
            points.emplace_back(std::pair<double, double>{point.position.x, point.position.y});
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

    farmbot_interfaces::msg::Segments vector2Segments(const std::vector<farmtrax::Swath>& swaths) {
        farmbot_interfaces::msg::Segments segments;
        for (const auto& swath : swaths) {
            farmbot_interfaces::msg::Segment segment;
            if (swath.direction == farmtrax::Direction::REVERSE) {
                segment.origin.pose.position.x = swath.swath[1].x();
                segment.origin.pose.position.y = swath.swath[1].y();
                segment.destination.pose.position.x = swath.swath[0].x();
                segment.destination.pose.position.y = swath.swath[0].y();
            } else {
                segment.origin.pose.position.x = swath.swath[0].x();
                segment.origin.pose.position.y = swath.swath[0].y();
                segment.destination.pose.position.x = swath.swath[1].x();
                segment.destination.pose.position.y = swath.swath[1].y();
            }
            segments.segments.push_back(segment);
        }
        return segments;
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
            if (swath.direction == farmtrax::Direction::REVERSE) {
                std::swap(p1, p2);
            }
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
    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), e.what());
    }
    rclcpp::shutdown();
    return 0;
}
