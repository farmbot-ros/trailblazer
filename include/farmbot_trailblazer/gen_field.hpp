#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <json/json.h>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

#include "farmbot_interfaces/srv/field.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "farmbot_trailblazer/utils/geojson.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace trailblazer {
    using Field = farmbot_interfaces::srv::Field;
    using namespace std::chrono_literals;
    using namespace std::placeholders;

    class GenField {
      private:
        rclcpp::Node::SharedPtr node_;
        std::string geojson_file_;
        rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
        std::vector<std::vector<double>> field_points_;
        std::vector<std::vector<double>> geojson_points_;

      public:
        GenField(rclcpp::Node::SharedPtr node) : node_(node) {
            geojson_file_ = node_->get_parameter_or<std::string>("geojson_file", "field.geojson");
            gps2enu_client_ = node_->create_client<farmbot_interfaces::srv::Gps2Enu>("loc/gps2enu");
            RCLCPP_INFO(node_->get_logger(), "GetTheField Service Node is ready.");
        }

        void gen_field() {
            getPointsFromGeoJSON(geojson_file_);
            std::thread([this] { navToEnu(geojson_points_); }).detach();
        }

        std::vector<std::vector<double>> get_field() { return field_points_; }

      private:
        void getPointsFromGeoJSON(const std::string &geojson_file) {
            try {
                auto geojsonObject = geojson::parseGeoJSONFromFile(geojson_file);
                geojson_points_ = geojson::utils::extractFirstPolygon(geojsonObject);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "Error parsing GeoJSON: %s", e.what());
            }
        }

        void navToEnu(const std::vector<std::vector<double>> &navpts) {
            auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();
            for (const auto &point : navpts) {
                sensor_msgs::msg::NavSatFix gps_point;
                gps_point.latitude = point[0];
                gps_point.longitude = point[1];
                gps_point.altitude = 0.0; // Adjust if altitude data is available
                request->gps.push_back(gps_point);
            }
            while (!gps2enu_client_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
            }
            // Send the request and wait for the result (blocking call)
            auto result_future = gps2enu_client_->async_send_request(request);
            while (rclcpp::ok() && result_future.wait_for(1s) == std::future_status::timeout) {
                RCLCPP_INFO(node_->get_logger(), "Waiting for response from GPS2ENU service...");
            }
            auto result = result_future.get();
            if (!result) {
                RCLCPP_ERROR(node_->get_logger(), "Service call failed.");
                return;
            }
            auto getres = result->enu;
            for (uint i = 0; i < getres.size(); i++) {
                field_points_.push_back({getres[i].position.x, getres[i].position.y, getres[i].position.z, navpts[i][0],
                                         navpts[i][1], navpts[i][2]});
            }
            RCLCPP_INFO(node_->get_logger(), "Successfully retrieved %zu waypoints.", field_points_.size());
        }
    };
} // namespace trailblazer
