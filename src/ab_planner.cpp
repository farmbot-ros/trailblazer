#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include <iostream>
#include "fields2cover.h"
#include "farmbot_planner/utils/geojson.hpp"
#include "farmbot_planner/utils/fieldgen.hpp"

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

    field::Field field_;
    farmbot_interfaces::msg::Segments segments_;
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PolygonStamped outer_polygon_;
    geometry_msgs::msg::PolygonStamped inner_polygon_;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

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

        name = this->get_parameter_or<std::string>("name", "ab_planner");
        topic_prefix_param = this->get_parameter_or<std::string>("topic_prefix", "/fb");
        geojson_file_ = this->get_parameter_or<std::string>("geojson_file", "field.geojson");
        RCLCPP_INFO(this->get_logger(), "GeoJSON file: %s", geojson_file_.c_str());
        vehicle_width_ = this->get_parameter_or<double>("vehicle_width", 0.5);
        vehicle_coverage_ = this->get_parameter_or<double>("vehicle_coverage", 3.0);
        path_spacing_ = this->get_parameter_or<double>("path_spacing", 0.1);
        path_angle_ = this->get_parameter_or<double>("path_angle", 90);

        field_ = field::Field(vehicle_width_, vehicle_coverage_, path_spacing_, path_angle_);

        // Create the service client
        gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>(topic_prefix_param + "/loc/gps2enu");

        // Create the path publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_prefix_param + "/pla/path", 10);
        inner_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_prefix_param + "/pla/inner_field", 10);
        outer_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_prefix_param + "/pla/outer_field", 10);

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

        auto path = field_.gen_path(enu_points);
        farmbot_interfaces::msg::Segments segments = field_.gen_segments(path);
        path_ = field_.gen_path_msg(path);
        std::tie(outer_polygon_, inner_polygon_) = field_.get_polygons();
        RCLCPP_INFO(this->get_logger(), "Generated %lu segments.", segments.segments.size());
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
