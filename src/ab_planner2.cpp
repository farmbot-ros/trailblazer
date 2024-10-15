#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include <iostream>
#include "farmbot_planner/utils/geojson.hpp"
#include "farmbot_planner/farmtrax/field.hpp"
#include "farmbot_planner/farmtrax/swath.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"

#include "farmbot_interfaces/msg/waypoint.hpp"
#include "farmbot_interfaces/msg/segment.hpp"
#include "farmbot_interfaces/msg/segments.hpp"


using namespace std::chrono_literals;

struct PathSegment {
    std::pair<double, double> start;
    std::pair<double, double> end;
    std::vector<std::pair<double, double>> middle;
};

class FieldProcessorNode : public rclcpp::Node {
private:
    std::string name;
    std::string topic_prefix_param;
    std::string geojson_file_;
    double vehicle_width_;
    double vehicle_coverage_;
    double path_spacing_;
    double path_angle_;

    farmtrax::Field field_;
    farmtrax::Swaths swaths_;

    farmbot_interfaces::msg::Segments segments_;
    nav_msgs::msg::Path path_;
    visualization_msgs::msg::Marker line_marker_;
    geometry_msgs::msg::PolygonStamped outer_polygon_;
    geometry_msgs::msg::PolygonStamped inner_polygon_;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr outer_polygon_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr inner_polygon_publisher_;

    bool service_started_ = false;
    rclcpp::TimerBase::SharedPtr service_start_timer_;
    bool planner_initialized_ = false;
    rclcpp::TimerBase::SharedPtr planner_init_timer_;


public:
    FieldProcessorNode() : Node("ab_planner",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("farmbot_planner");
        std::string geojson_path = package_share_directory + "/config/field.geojson";

        name = this->get_parameter_or<std::string>("name", "ab_planner");
        topic_prefix_param = this->get_parameter_or<std::string>("topic_prefix", "/fb");
        geojson_file_ = this->get_parameter_or<std::string>("geojson_file", geojson_path);
        RCLCPP_INFO(this->get_logger(), "GeoJSON file: %s", geojson_file_.c_str());
        vehicle_width_ = this->get_parameter_or<double>("vehicle_width", 0.5);
        vehicle_coverage_ = this->get_parameter_or<double>("vehicle_coverage", 3.0);
        path_spacing_ = this->get_parameter_or<double>("path_spacing", 0.1);
        path_angle_ = this->get_parameter_or<double>("path_angle", 90);

        // Create the service client
        gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>(topic_prefix_param + "/loc/gps2enu");

        // Create the path publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_prefix_param + "/pla/path", 10);
        inner_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_prefix_param + "/pla/inner_field", 10);
        outer_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_prefix_param + "/pla/outer_field", 10);
        line_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_prefix_param + "/pla/swath_lines", 10);

        // Timers
        service_start_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FieldProcessorNode::on_service_start_timer, this));
        planner_init_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FieldProcessorNode::on_planner_init_timer, this));
    }

private:
    void on_service_start_timer() {
        if (!gps2enu_client_->wait_for_service(10s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for GPS to ENU conversion service...");
        } else {
            service_started_ = true;
            path_publisher_->publish(path_);
            outer_polygon_publisher_->publish(outer_polygon_);
            inner_polygon_publisher_->publish(inner_polygon_);
            line_publisher_->publish(line_marker_);
        }
    }

    void on_planner_init_timer() {
        if (!service_started_) { return; }
        std::vector<std::vector<double>> points = getPointsFromGeoJSON(geojson_file_);
        if (points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No points found in the GeoJSON file.");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Processing %lu points from the GeoJSON file.", points.size());
        }
        generate_field(points);
    }


    void generate_field(std::vector<std::vector<double>> points) {
        planner_initialized_ = true;
        planner_init_timer_->cancel();
        // Use the first point as the reference for the datum
        sensor_msgs::msg::NavSatFix ref_point;
        ref_point.latitude = points[0][0];
        ref_point.longitude = points[0][1];
        ref_point.altitude = 0.0;  // Set to 0 or any relevant altitude
        // Prepare the request for GPS to ENU conversion
        auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();
        request->datum = ref_point;
        // Add the rest of the points to the request
        for (const auto& point : points) {
            sensor_msgs::msg::NavSatFix gps_point;
            gps_point.latitude = point[0];
            gps_point.longitude = point[1];
            gps_point.altitude = 0.0;  // Adjust if altitude data is available
            request->gps.push_back(gps_point);
        }
        // Send the request to the service
        if (!gps2enu_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for GPS to ENU conversion service...");
        } 
        // Send the request to the service
        auto state_result = gps2enu_client_->async_send_request(request, [this, request, points](rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedFuture future) {
            auto response = future.get();
            process_enu_points(response->enu);
        });
        planner_init_timer_->cancel();
    }

    std::vector<std::vector<double>> getPointsFromGeoJSON(const std::string& geojson_file) {
        std::vector<std::vector<double>> points;
        try {
            auto geojsonObject = geojson::parseGeoJSONFromFile(geojson_file);
            points = geojson::utils::extractFirstPolygon(geojsonObject);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing GeoJSON: %s", e.what());
        }
        return points;
    }

    void process_enu_points(const std::vector<geometry_msgs::msg::Pose>& enu_points) {
        if (enu_points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No ENU points received.");
            return;
        }

        std::vector<std::pair<double, double>> points;
        for (const auto& point : enu_points) {
            points.emplace_back(point.position.x, point.position.y);
        }

        // auto path = field_.gen_path(enu_points);
        // field_.update_route(-293.946, 208.87);
        // path_ = field_.gen_path_msg();
        // farmbot_interfaces::msg::Segments segments = field_.gen_segments();
        // std::tie(outer_polygon_, inner_polygon_) = field_.get_polygons();
        // RCLCPP_INFO(this->get_logger(), "Generated %lu segments.", segments.segments.size());

        field_.setBoundary(points);
        farmtrax::Field hl = field_.getShrunkField(vehicle_width_ * 6.0);
        // auto hl = field_.generateHeadlands(vehicle_width_);

        outer_polygon_ = vector2Polygon(field_.getBoundary());
        inner_polygon_ = vector2Polygon(hl.getBoundary());

        swaths_ = farmtrax::Swaths(field_, hl, 10.0);
        swaths_.connectSwathsInUShape();
        path_ = vector2Path(swaths_.getSwaths());
        line_marker_ = vector2Lines(swaths_.getSwaths());


        for (const auto& point : points) {
            RCLCPP_INFO(this->get_logger(), "ENU Point: %f, %f", point.first, point.second);
        }

    }

    // vector of vector of double to ros polygon
    geometry_msgs::msg::PolygonStamped vector2Polygon(const std::vector<std::vector<double>>& points) {
        geometry_msgs::msg::PolygonStamped polygon;
        polygon.header.frame_id = "map";
        polygon.header.stamp = rclcpp::Clock().now();
        for (const auto& point : points) {
            geometry_msgs::msg::Point32 p;
            p.x = point[0];
            p.y = point[1];
            polygon.polygon.points.push_back(p);
        }
        return polygon;
    }


    nav_msgs::msg::Path vector2Path(const std::vector<farmtrax::Swath>& swaths) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
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

    visualization_msgs::msg::Marker vector2Lines(const std::vector<farmtrax::Swath>& swaths) {
        visualization_msgs::msg::Marker line_marker;
        
        // Set the frame and timestamp
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = rclcpp::Clock().now();

        // Set the namespace and id for this marker
        line_marker.ns = "swath_lines";
        line_marker.id = 0;

        // Define the type of marker (LINE_STRIP)
        line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;

        // Set the scale of the lines (width of the lines)
        line_marker.scale.x = 0.1;  // Adjust the width of the line here

        // Set the color (RGBA)


        // Set the lifetime of the marker (0 means forever)
        line_marker.lifetime = rclcpp::Duration::from_seconds(0);

        // Iterate through the swaths and add points to the marker
        for (const auto& swath : swaths) {
            geometry_msgs::msg::Point p;
            for (const auto& point : swath.swath) {
                p.x = point.x();
                p.y = point.y();
                p.z = 0.0;  // Set the z-coordinate to 0
                if (swath.type == farmtrax::SwathType::LAND) {
                    line_marker.color.r = 0.0f;  // Green
                    line_marker.color.g = 1.0f;
                    line_marker.color.b = 0.0f;
                    line_marker.color.a = 1.0f;  // Fully opaque
                } else {
                    line_marker.color.r = 1.0f;  // Red
                    line_marker.color.g = 0.0f;
                    line_marker.color.b = 0.0f;
                    line_marker.color.a = 1.0f;  // Fully opaque
                }
                line_marker.points.push_back(p);
            }
        }

        return line_marker;
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