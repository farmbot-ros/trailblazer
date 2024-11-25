#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <json/json.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "farmbot_interfaces/srv/get_the_field.hpp"
#include "farmbot_planner/utils/geojson.hpp"
#include "geometry_msgs/msg/point.hpp"

using GetTheField = farmbot_interfaces::srv::GetTheField;
using namespace std::chrono_literals;
using namespace std::placeholders;

class GetTheFieldService : public rclcpp::Node {
    private:
        std::string geojson_file_;
        rclcpp::Service<GetTheField>::SharedPtr service_;
        rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;

        // Add callback groups
        rclcpp::CallbackGroup::SharedPtr client_callback_group_;
        rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    public:
        GetTheFieldService() : Node("getthe_field") {

            std::string package_share_directory = ament_index_cpp::get_package_share_directory("farmbot_planner");
            std::string geojson_path = package_share_directory + "/config/field.geojson";

            geojson_file_ = this->get_parameter_or<std::string>("geojson_file", geojson_path);

            // Create callback groups
            client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // Create the GPS to ENU client, assign it to the client callback group
            gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>(
                "loc/gps2enu",
                rmw_qos_profile_services_default,
                client_callback_group_);

            // Create the service, assign it to the service callback group
            service_ = this->create_service<GetTheField>(
                "pln/get_field",
                std::bind(&GetTheFieldService::handle_get_the_field, this, _1, _2),
                rmw_qos_profile_services_default,
                service_callback_group_);

            RCLCPP_INFO(this->get_logger(), "GetTheField Service Node is ready.");
        }

    private:
        void handle_get_the_field(
            const std::shared_ptr<GetTheField::Request> request,
            std::shared_ptr<GetTheField::Response> response) {

            RCLCPP_INFO(this->get_logger(), "Received GetTheField request.");
            std::string geojson_file = request->geojson_file;
            if (geojson_file.empty()) {
                geojson_file = geojson_file_;
            }
            std::vector<sensor_msgs::msg::NavSatFix> waypoints;
            std::vector<std::vector<double>> loc_points;
            auto points = getPointsFromGeoJSON(geojson_file);
            waypoints.reserve(points.size());
            for (const auto& point : points) {
                if (point.size() < 2) {
                    RCLCPP_WARN(this->get_logger(), "Invalid coordinate format.");
                    continue;
                }
                double lat = point[0];
                double lon = point[1];
                sensor_msgs::msg::NavSatFix waypoint;
                waypoint.latitude = lat;
                waypoint.longitude = lon;
                waypoint.altitude = 0.0; // Altitude is not provided by GeoJSON
                waypoints.push_back(waypoint);
                loc_points.push_back({lat, lon});
            }
            response->field = waypoints;
            auto enu_points = nav_to_enu(loc_points);
            response->points = enu_points;
            RCLCPP_INFO(this->get_logger(), "Successfully retrieved %zu waypoints.", waypoints.size());
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

        std::vector<geometry_msgs::msg::Point> nav_to_enu(const std::vector<std::vector<double>>& navpts) {
            std::vector<geometry_msgs::msg::Point> points;
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
            // Send the request and wait for the result (blocking call)
            auto result_future = gps2enu_client_->async_send_request(request);
            auto result = result_future.get();
            if (!result) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed.");
                return {};
            }
            auto getres = result->enu;
            for (const auto& point : getres) {
                points.push_back(point.position);
            }
            return points;
        }
};

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<GetTheFieldService>();

    // Use a MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
