#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <json/json.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "farmbot_interfaces/srv/get_the_field.hpp"
#include "farmbot_planner/utils/geojson.hpp"


using GetTheField = farmbot_interfaces::srv::GetTheField;

class GetTheFieldService : public rclcpp::Node{
    private:
        std::string name;
        std::string topic_prefix_param;
        std::string geojson_file_;
        rclcpp::Service<GetTheField>::SharedPtr service_;

    public:
        GetTheFieldService(): Node("getthe_field"){
            name = this->get_parameter_or<std::string>("name", "getthe_field");
            topic_prefix_param = this->get_parameter_or<std::string>("topic_prefix", "/fb");

            std::string package_share_directory = ament_index_cpp::get_package_share_directory("farmbot_planner");
            std::string geojson_path = package_share_directory + "/config/field.geojson";

            geojson_file_ = this->get_parameter_or<std::string>("geojson_file", geojson_path);

            // Create the service
            service_ = this->create_service<GetTheField>(
                topic_prefix_param + "/pla/get_field",
                std::bind(&GetTheFieldService::handle_get_the_field, this, std::placeholders::_1, std::placeholders::_2)
            );

            RCLCPP_INFO(this->get_logger(), "GetTheField Service Node is ready.");
        }

    private:
        void handle_get_the_field(const std::shared_ptr<GetTheField::Request> request, std::shared_ptr<GetTheField::Response> response){
            RCLCPP_INFO(this->get_logger(), "Received GetTheField request.");
            std::string geojson_file = request->geojson_file;
            if (geojson_file.empty()) {
                geojson_file = geojson_file_;
            }
            std::vector<sensor_msgs::msg::NavSatFix> waypoints;
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
            }
            response->field = waypoints;
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
};

int main(int argc, char** argv){
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    // Create and spin the node
    auto node = std::make_shared<GetTheFieldService>();
    rclcpp::spin(node);
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
