#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <json/json.h>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "farmbot_interfaces/srv/field.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "farmbot_trailblazer/utils/geojson.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace trailblazer {
    using Field = farmbot_interfaces::srv::Field;
    using namespace std::chrono_literals;
    using namespace std::placeholders;
    namespace echo = spdlog;

    class GetField {
      private:
        rclcpp::Node::SharedPtr node_;
        std::string geojson_file_;
        rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;

        // Add callback groups
        rclcpp::CallbackGroup::SharedPtr client_callback_group_;

      public:
        GetField() = default;

        void init(rclcpp::Node::SharedPtr node) {
            node_ = node;
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("farmbot_trailblazer");
            std::string geojson_path = package_share_directory + "/config/field.geojson";

            geojson_file_ = node_->get_parameter_or<std::string>("geojson_file", geojson_path);

            echo::info("GeoJSON file: {}", geojson_file_);

            // Create callback groups
            client_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            // rclcpp::QoS qos_profile(10);
            auto qos_profile = rmw_qos_profile_t();

            // Create the GPS to ENU client, assign it to the client callback group
            gps2enu_client_ = node_->create_client<farmbot_interfaces::srv::Gps2Enu>("loc/gps2enu", qos_profile,
                                                                                     client_callback_group_);

            RCLCPP_INFO(node_->get_logger(), "GetTheField Service Node is ready.");
        }

        std::vector<std::vector<double>> getBorders() {
            auto points = getPointsFromGeoJSON(geojson_file_);
            return navToEnu(points);
        }

      private:
        std::vector<std::vector<double>> getPointsFromGeoJSON(const std::string &geojson_file) {
            std::vector<std::vector<double>> points;
            try {
                auto geojsonObject = geojson::parseGeoJSONFromFile(geojson_file);
                points = geojson::utils::extractFirstPolygon(geojsonObject);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "Error parsing GeoJSON: %s", e.what());
            }
            return points;
        }

        std::vector<std::vector<double>> navToEnu(const std::vector<std::vector<double>> &navpts) {
            std::vector<std::vector<double>> points;
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
                    return {};
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
                return {};
            }
            auto getres = result->enu;
            for (uint i = 0; i < getres.size(); i++) {
                points.push_back({getres[i].position.x, getres[i].position.y, getres[i].position.z, navpts[i][0],
                                  navpts[i][1], navpts[i][2]});
            }
            return points;
        }
    };
} // namespace trailblazer
