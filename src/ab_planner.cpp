#include "fields2cover.h"
#include <iostream>
#include "farmbot_planner/utils/geojson.hpp"

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"

#include "farmbot_interfaces/msg/waypoint.hpp"
#include "farmbot_interfaces/msg/segment.hpp"
#include "farmbot_interfaces/msg/segments.hpp"

#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>

using namespace std::chrono_literals;

struct PathSegment {
    std::pair<double, double> start;
    std::pair<double, double> end;
    std::vector<std::pair<double, double>> middle;
};

class FieldProcessorNode : public rclcpp::Node {
private:
    std::string geojson_file_;
    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    bool service_started_ = false;
    rclcpp::TimerBase::SharedPtr service_start_timer_;
    bool planner_initialized_ = false;
    rclcpp::TimerBase::SharedPtr planner_init_timer_;

    farmbot_interfaces::msg::Segment segment_;

    // Moved variables
    F2CLinearRing ring_;
    F2CCells cells_;
    F2CRobot robot_;
    f2c::pp::PathPlanning path_planner_;
    f2c::pp::DubinsCurves dubins_;
    double finness_;

public:
    FieldProcessorNode() : Node("field_processor_node") {
        // Declare and get parameters
        this->declare_parameter<std::string>("geojson_file", "/home/bresilla/FARMBOT/src/farmbot_planner/config/field.geojson");
        this->get_parameter("geojson_file", geojson_file_);

        robot_.setWidth(0.5);
        robot_.setCovWidth(5.0);

        finness_ = 0.5;

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
        ref_point.latitude = points[0][0];
        ref_point.longitude = points[0][1];
        ref_point.altitude = 0.0;  // Set to 0 or any relevant altitude

        // Prepare the request for GPS to ENU conversion
        auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();
        request->datum = ref_point;

        for (const auto& point : points) {
            sensor_msgs::msg::NavSatFix gps_point;
            gps_point.latitude = point[0];
            gps_point.longitude = point[1];
            gps_point.altitude = 0.0;  // Adjust if altitude data is available
            request->gps.push_back(gps_point);
        }

        if (!gps2enu_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for GPS to ENU conversion service...");
        } 

        auto state_result = gps2enu_client_->async_send_request(request, [this, request, points](rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedFuture future) {
            auto response = future.get();
            process_enu_points(response->enu);
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
        }

        cells_ = F2CCells(F2CCell{ring_});

        f2c::hg::ConstHL const_hl;
        F2CCells no_hl = const_hl.generateHeadlands(cells_, 3.0 * robot_.getWidth());

        f2c::sg::BruteForce bf;
        F2CSwathsByCells swaths = bf.generateSwaths(M_PI/2.0, robot_.getCovWidth(), no_hl);

        f2c::rp::RoutePlannerBase route_planner;
        F2CRoute route = route_planner.genRoute(no_hl, swaths);

        for (auto& line : route.getVectorSwaths()) {
            for (auto& sw : line) {
                double x1 = sw.startPoint().X();
                double y1 = sw.startPoint().Y();
                double x2 = sw.endPoint().X();
                double y2 = sw.endPoint().Y();
                // std::cout << "Swath: " << x1 << ", " << y1 << " -> " << x2 << ", " << y2 << std::endl;
            }
        }

        f2c::pp::DubinsCurves dubins;
        F2CPath path = path_planner_.planPath(robot_, route, dubins);
        path.reduce(finness_);

        farmbot_interfaces::msg::Segments segments;
        std::vector<PathSegment> pathSegments = pathSegmentGen(path);

        for (const auto& segment : pathSegments) {
            farmbot_interfaces::msg::Segment seg;
            seg.origin.pose.position.x = segment.start.first;
            seg.origin.pose.position.y = segment.start.second;
            seg.destination.pose.position.x = segment.end.first;
            seg.destination.pose.position.y = segment.end.second;
            for (const auto& middle_point : segment.middle) {
                farmbot_interfaces::msg::Waypoint middle;
                middle.pose.position.x = middle_point.first;
                middle.pose.position.y = middle_point.second;
                seg.inbetween.push_back(middle);
            }
            segments.segments.push_back(seg);
        }

        for (const auto& segment : pathSegments) {
            std::cout << "Segment: " << segment.start.first << ", " << segment.start.second << " -> " << segment.end.first << ", " << segment.end.second << std::endl;
            for (const auto& middle_point : segment.middle) {
                std::cout << "  Middle: " << middle_point.first << ", " << middle_point.second << std::endl;
            }
        }


        f2c::Visualizer::figure();
        f2c::Visualizer::plot(cells_);
        f2c::Visualizer::plot(no_hl);
        f2c::Visualizer::plot(path);
        f2c::Visualizer::figure_size(2400, 2400);
        f2c::Visualizer::save("Tutorial_8_1_ENU.png");

        RCLCPP_INFO(this->get_logger(), "Field processing completed and visualization saved.");
    }

    std::vector<PathSegment> pathSegmentGen(const F2CPath& path) {
        std::vector<PathSegment> pathSegments;
        PathSegment segm;
        auto last_state_type = f2c::types::PathSectionType::HL_SWATH;
        f2c::types::PathState prevState; // Define the previous state properly
        for (const auto& state : path) {
            if (state.type == f2c::types::PathSectionType::SWATH) {
                // If the last state was a TURN, complete the segment and store it
                if (last_state_type == f2c::types::PathSectionType::TURN) {
                    segm.start = segm.middle.front();
                    segm.middle.erase(segm.middle.begin());
                    segm.end = {state.point.X(), state.point.Y()};
                    pathSegments.push_back(segm);
                    // Clear the segment for the next one
                    segm = PathSegment();
                    segm.middle.clear();
                }
                // Store the SWATH start point
            } else if (state.type == f2c::types::PathSectionType::TURN) {
                // If the last state was a SWATH, start a new segment
                if (last_state_type == f2c::types::PathSectionType::SWATH) {
                    segm.start = {prevState.point.X(), prevState.point.Y()};
                    segm.end = {state.point.X(), state.point.Y()};
                    pathSegments.push_back(segm);
                    // Clear the segment for the next one
                    segm = PathSegment();
                    segm.middle.clear();
                }
                // Add the TURN points to the middle vector
                segm.middle.push_back({state.point.X(), state.point.Y()});
            }
            // Update the previous state and last state type
            prevState = state;
            last_state_type = state.type;
        }
        // Make sure to handle the last segment if it exists
        if (!segm.middle.empty() || last_state_type == f2c::types::PathSectionType::SWATH) {
            pathSegments.push_back(segm);
        }
        return pathSegments;
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
