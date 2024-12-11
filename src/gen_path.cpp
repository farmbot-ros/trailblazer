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
    bool route_initialized_ = false;
    bool received_swaths_ = false;

    std::string namespace_;

    farmtrax::Mesh mesh_;
    farmtrax::Route route_;

    visualization_msgs::msg::MarkerArray graph_markers_;
    nav_msgs::msg::Path path_;

    rclcpp::TimerBase::SharedPtr router_timer_;

    farmbot_interfaces::msg::Segments segments_;
    rclcpp::Publisher<farmbot_interfaces::msg::Segments>::SharedPtr segment_publisher_;

    std::vector<farmtrax::Swath> swaths_vec_;
    rclcpp::Subscription<farmbot_interfaces::msg::Swaths>::SharedPtr swaths_subscriber_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graphviz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

public:
    FieldProcessorNode() : Node("gen_path",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {

        // Create publishers
        graphviz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pln/graph", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("pln/path", 10);

        // Segment publisher
        segment_publisher_ = this->create_publisher<farmbot_interfaces::msg::Segments>("pln/segments", 10);

        // Swaths subscriber
        swaths_subscriber_ = this->create_subscription<farmbot_interfaces::msg::Swaths>("/pln/swaths", 10, std::bind(&FieldProcessorNode::swaths_callback, this, _1));

        // Timers
        router_timer_ = this->create_wall_timer(1s, std::bind(&FieldProcessorNode::router_timer_cb, this));

        // Namespace
        namespace_ = this->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }

    }

    void init() {
        route_.pass_node(this->shared_from_this());
        mesh_.pass_node(this->shared_from_this());
        // Run the main processing in a separate thread
        std::thread([this] {
            gen_route();
        }).detach();
    }

private:

    void router_timer_cb() {
        if (!route_initialized_) { return; }
        path_publisher_->publish(path_);
        graphviz_pub_->publish(graph_markers_);
        segment_publisher_->publish(segments_);
    }

    void swaths_callback(const farmbot_interfaces::msg::Swaths::SharedPtr msg) {
        std::vector<farmtrax::Swath> swaths = swathMsg2Swath(msg);
        if (!swaths.empty()) {
            swaths_vec_ = swaths;
            received_swaths_ = true;
        }
    }

    void gen_route() {
        while (!received_swaths_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for swaths");
            std::this_thread::sleep_for(1s);
        }
        mesh_.build_graph(swaths_vec_);
        graph_markers_ =  graph2Markers(mesh_);
        RCLCPP_INFO (this->get_logger(), "Graph generated");

        route_.find_optimal(mesh_, farmtrax::Route::Algorithm::EXHAUSTIVE_SEARCH);
        path_ = vector2Path(route_.get_swaths());
        segments_ = vector2Segments(route_.get_swaths());
        route_initialized_ = true;
        // RCLCPP_INFO(this->get_logger(), "Route generated: %lu", route_.get_swaths().size());
    }

    std::vector<farmtrax::Swath> swathMsg2Swath(const farmbot_interfaces::msg::Swaths::SharedPtr msg) {
        std::vector<farmtrax::Swath> swaths;
        for (const auto& swath_msg : msg->swaths) {
            farmtrax::Swath swath;
            if (swath_msg.robot.data == namespace_){
                for (const auto& point : swath_msg.line.points) {
                    swath.swath.push_back(farmtrax::Point(point.x, point.y));
                }
                swath.length = swath_msg.length.data;
                swath.uuid = swath_msg.uuid.data;
                swath.type = static_cast<farmtrax::SwathType>(swath_msg.type.data);
                swaths.push_back(swath);
            }
        }
        return swaths;
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

    nav_msgs::msg::Path vector2Path(const std::vector<farmtrax::Swath>& swaths) {
        nav_msgs::msg::Path path;
        path.header.frame_id = namespace_ + "/map";
        path.header.stamp = rclcpp::Clock().now();
        for (const auto& swath : swaths) {
            for (const auto& point : swath.swath) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
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
