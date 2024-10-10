#include "fields2cover.h"
#include <iostream>
#include "farmbot_planner/utils/geojson.hpp"

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"

#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>

using namespace std::chrono_literals;

class FieldProcessorNode : public rclcpp::Node {
private:
    std::string geojson_file_;
    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    bool service_started_ = false;
    rclcpp::TimerBase::SharedPtr service_start_timer_;
    bool planner_initialized_ = false;
    rclcpp::TimerBase::SharedPtr planner_init_timer_;

    // Moved variables
    F2CLinearRing ring_;
    F2CCells cells_;
    F2CRobot robot_;
    f2c::pp::PathPlanning path_planner_;
    f2c::pp::DubinsCurves dubins_;

public:
    FieldProcessorNode() : Node("field_processor_node"), robot_(0.5, 1) {
        // Declare and get parameters
        this->declare_parameter<std::string>("geojson_file", "/home/bresilla/FARMBOT/src/farmbot_planner/config/field.geojson");
        this->get_parameter("geojson_file", geojson_file_);

        // Create the service client
        gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>("/fb/loc/gps2enu");

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
            service_start_timer_->cancel();
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

        planner_initialized_ = true;
        planner_init_timer_->cancel();

        // Use the first point as the reference for the datum
        sensor_msgs::msg::NavSatFix ref_point;
        ref_point.latitude = points[0][1];
        ref_point.longitude = points[0][0];
        ref_point.altitude = 0.0;  // Set to 0 or any relevant altitude

        // Prepare the request for GPS to ENU conversion
        auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();
        request->datum = ref_point;

        for (const auto& point : points) {
            sensor_msgs::msg::NavSatFix gps_point;
            gps_point.latitude = point[1];
            gps_point.longitude = point[0];
            gps_point.altitude = 0.0;  // Adjust if altitude data is available
            request->gps.push_back(gps_point);
        }

        if (!gps2enu_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for GPS to ENU conversion service...");
        } 

        auto state_result = gps2enu_client_->async_send_request(request, [this, request](rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedFuture future) {
            auto response = future.get();
            process_enu_points(response->enu);
            RCLCPP_INFO(this->get_logger(), "Field processing completed.");
        });
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

        ring_ = F2CLinearRing();
        for (const auto& enu_point : enu_points) {
            ring_.addPoint(F2CPoint(enu_point.position.x, enu_point.position.y));
            RCLCPP_INFO(this->get_logger(), "Transformed Point: [%f, %f]", enu_point.position.x, enu_point.position.y);
        }

        cells_ = F2CCells(F2CCell{ring_});

        f2c::hg::ConstHL const_hl;
        F2CCells no_hl = const_hl.generateHeadlands(cells_, 3.0 * robot_.getWidth());

        f2c::sg::BruteForce bf;
        F2CSwaths swaths = bf.generateSwaths(M_PI / 1.32, robot_.getCovWidth(), no_hl.getGeometry(0));

        f2c::rp::SnakeOrder snake_sorter;
        swaths = snake_sorter.genSortedSwaths(swaths);

        robot_.setMinTurningRadius(2);  // m
        F2CPath path = path_planner_.planPath(robot_, swaths, dubins_);

        f2c::Visualizer::figure();
        f2c::Visualizer::plot(cells_);
        f2c::Visualizer::plot(no_hl);
        f2c::Visualizer::plot(path);
        f2c::Visualizer::figure_size(2400, 2400);
        f2c::Visualizer::save("Tutorial_8_1_ENU.png");

        RCLCPP_INFO(this->get_logger(), "Field processing completed and visualization saved.");
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
