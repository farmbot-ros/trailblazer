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
#include "geometry_msgs/msg/point.hpp"

namespace echo = spdlog;
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

    geometry_msgs::msg::PolygonStamped outer_polygon_;
    geometry_msgs::msg::PolygonStamped inner_polygon_;
    visualization_msgs::msg::MarkerArray field_arrows_;
    visualization_msgs::msg::MarkerArray graph_markers_;
    nav_msgs::msg::Path path_;

    rclcpp::TimerBase::SharedPtr visualization_timer_;
    rclcpp::Client<farmbot_interfaces::srv::GetTheField>::SharedPtr get_the_field_client_;

    rclcpp::TimerBase::SharedPtr on_timer_;
    farmbot_interfaces::msg::Segments segments_;
    rclcpp::Publisher<farmbot_interfaces::msg::Segments>::SharedPtr segment_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr field_arrows_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graphviz_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr outer_polygon_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr inner_polygon_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

public:
    FieldProcessorNode() : Node("ab_planner",
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
        graphviz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pln/graph", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("pln/path", 10);

        // Segment publisher
        segment_publisher_ = this->create_publisher<farmbot_interfaces::msg::Segments>("pln/segments", 10);
        on_timer_ = this->create_wall_timer(1s, std::bind(&FieldProcessorNode::on_timer, this));

        // Timers
        visualization_timer_ = this->create_wall_timer(1s, std::bind(&FieldProcessorNode::on_service_start_timer, this));

        // Namespace
        namespace_ = this->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }

        // Run the main processing in a separate thread
        std::thread([this] { runner(); }).detach();
    }

    void init() {
        swaths_.pass_node(this->shared_from_this());
        route_.pass_node(this->shared_from_this());
        field_.pass_node(this->shared_from_this());
        mesh_.pass_node(this->shared_from_this());
    }

private:
    void on_service_start_timer() {
        if (!planner_initialized_) { return; }
        outer_polygon_publisher_->publish(outer_polygon_);
        inner_polygon_publisher_->publish(inner_polygon_);
        field_arrows_pub_->publish(field_arrows_);
        graphviz_pub_->publish(graph_markers_);
        path_publisher_->publish(path_);
    }

    void on_timer() {
        if (!planner_initialized_) { return; }
        segment_publisher_->publish(segments_);
    }

    void runner() {
        planner_initialized_ = true;
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

        swaths_.gen_swaths(field_, hl, vehicle_coverage_, path_angle_, alternate_freq_, vehicle_width_);
        field_arrows_ = vector2ArrowsColor(swaths_.get_swaths());
        RCLCPP_INFO(this->get_logger(), "Swaths generated: %lu", swaths_.get_swaths().size());

        mesh_.build_graph(swaths_);
        graph_markers_ =  graph2Markers(mesh_);
        RCLCPP_INFO (this->get_logger(), "Graph generated");

        route_.find_optimal(mesh_, farmtrax::Route::Algorithm::EXHAUSTIVE_SEARCH);
        path_ = vector2Path(route_.get_swaths());
        segments_ = vector2Segments(route_.get_swaths());
        // RCLCPP_INFO(this->get_logger(), "Route generated: %lu", route_.get_swaths().size());
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

    farmbot_interfaces::msg::Segments vector2Segments(const std::vector<farmtrax::Swath>& swaths) {
        farmbot_interfaces::msg::Segments segments;
        for (const auto& swath : swaths) {
            farmbot_interfaces::msg::Segment segment;
            segment.origin.pose.position.x = swath.swath[0].x();
            segment.origin.pose.position.y = swath.swath[0].y();
            segment.destination.pose.position.x = swath.swath[1].x();
            segment.destination.pose.position.y = swath.swath[1].y();
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
            arrow.points.push_back(p1);
            arrow.points.push_back(p2);
            markers.markers.push_back(arrow);
        }
        return markers;
    }


    nav_msgs::msg::Path vector2Path(const std::vector<farmtrax::Swath>& swaths) {
        nav_msgs::msg::Path path;
        path.header.frame_id = namespace_ + "/map";
        path.header.stamp = rclcpp::Clock().now();
        for (const auto& swath : swaths) {
            for (const auto& point : swath.swath) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = point.x();
                pose.pose.position.y = point.y();
                path.poses.push_back(pose);
            }
        }
        return path;
    }


    visualization_msgs::msg::MarkerArray graph2Markers(const farmtrax::Mesh& mesh) {
        int id = 0;  // Unique ID for each marker
        visualization_msgs::msg::MarkerArray marker_array;

        // Publish vertices as spheres
        auto vertices = boost::vertices(mesh.graph_);
        for (auto vi = vertices.first; vi != vertices.second; ++vi) {
            auto v = *vi;
            const farmtrax::Point& p = mesh.graph_[v].point;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = namespace_ + "/map";
            marker.header.stamp = this->now();
            marker.ns = "vertices";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the pose of the marker
            marker.pose.position.x = p.x();
            marker.pose.position.y = p.y();
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Set the scale of the marker
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            // Set the color (blue)
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;

            marker.lifetime = rclcpp::Duration(0, 0);  // Lasts forever

            marker_array.markers.push_back(marker);
        }

        // Publish edges as lines
        auto edges = boost::edges(mesh.graph_);
        for (auto ei = edges.first; ei != edges.second; ++ei) {
            auto e = *ei;
            auto s = boost::source(e, mesh.graph_);
            auto t = boost::target(e, mesh.graph_);

            const farmtrax::Point& sp = mesh.graph_[s].point;
            const farmtrax::Point& tp = mesh.graph_[t].point;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = namespace_ + "/map";
            marker.header.stamp = this->now();
            marker.ns = "edges";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the scale of the marker
            marker.scale.x = 0.01;  // Line width

            // Set the color (violet)
            marker.color.r = 0.5f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;

            marker.lifetime = rclcpp::Duration(0, 0);  // Lasts forever

            // Set the start and end points of the line
            geometry_msgs::msg::Point start_point;
            start_point.x = sp.x();
            start_point.y = sp.y();
            start_point.z = 0.0;

            geometry_msgs::msg::Point end_point;
            end_point.x = tp.x();
            end_point.y = tp.y();
            end_point.z = 0.0;

            marker.points.push_back(start_point);
            marker.points.push_back(end_point);

            marker_array.markers.push_back(marker);
        }

        // Publish the entire marker array
        return marker_array;
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
