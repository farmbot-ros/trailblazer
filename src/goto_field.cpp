#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/srv/go_to_field.hpp"

#include <curl/curl.h>
#include <json/json.h> // Install jsoncpp or use another JSON library

using GetRoute = farmbot_interfaces::srv::GoToField;

// Callback function to handle data received by libcurl
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    std::string* response = static_cast<std::string*>(userp);
    size_t totalSize = size * nmemb;
    response->append(static_cast<char*>(contents), totalSize);
    return totalSize;
}

class OsrmRouteServiceNode : public rclcpp::Node{
    private:
        std::string name;
        std::string topic_prefix_param;
        rclcpp::Service<GetRoute>::SharedPtr service_;
        std::string osrm_server_url_;

    public:
        OsrmRouteServiceNode(): Node("goto_field"){
            name = this->get_parameter_or<std::string>("name", "goto_field");
            topic_prefix_param = this->get_parameter_or<std::string>("topic_prefix", "/fb");
            // Set the public OSRM server URL
            osrm_server_url_ = this->get_parameter_or<std::string>("osrm_server_url", "http://router.project-osrm.org");


            // Create the service
            service_ = this->create_service<GetRoute>(
                topic_prefix_param + "/pln/get_route",
                std::bind(&OsrmRouteServiceNode::handle_get_route, this, std::placeholders::_1, std::placeholders::_2)
            );

            RCLCPP_INFO(this->get_logger(), "OSRM Route Service Node is ready.");
        }

    private:
        void handle_get_route(const std::shared_ptr<GetRoute::Request> request, std::shared_ptr<GetRoute::Response> response){
            RCLCPP_INFO(this->get_logger(), "Received GetRoute request.");

            // Extract start and end coordinates
            double start_lat = request->start.latitude;
            double start_lon = request->start.longitude;
            double end_lat = request->end.latitude;
            double end_lon = request->end.longitude;

            // Construct the OSRM API URL using the public server
            std::ostringstream url_stream;
            url_stream << osrm_server_url_ << "/route/v1/foot/"
                    << start_lon << "," << start_lat << ";"
                    << end_lon << "," << end_lat
                    << "?geometries=geojson&overview=full";
            std::string url = url_stream.str();

            RCLCPP_DEBUG(this->get_logger(), "OSRM API URL: %s", url.c_str());

            // Initialize CURL
            CURL* curl = curl_easy_init();
            if (!curl) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize libcurl.");
            }

            std::string response_string;
            CURLcode res;

            // Set CURL options
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L); // Timeout after 10 seconds

            // Perform the request
            res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                RCLCPP_ERROR(this->get_logger(), "libcurl request failed: %s", curl_easy_strerror(res));
                curl_easy_cleanup(curl);
                return;
            }

            // Cleanup CURL
            curl_easy_cleanup(curl);

            // Parse JSON response
            Json::CharReaderBuilder reader_builder;
            Json::Value json_data;
            std::string errs;

            std::istringstream iss(response_string);
            if (!Json::parseFromStream(reader_builder, iss, &json_data, &errs)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON response: %s", errs.c_str());
                return;
            }

            // Check for 'routes' in JSON
            if (!json_data.isMember("routes") || !json_data["routes"].isArray() || json_data["routes"].empty()) {
                RCLCPP_ERROR(this->get_logger(), "No routes found in OSRM response.");
                return;
            }

            // Extract geometry coordinates
            const Json::Value& geometry = json_data["routes"][0]["geometry"];
            if (!geometry.isMember("coordinates") || !geometry["coordinates"].isArray()) {
                RCLCPP_ERROR(this->get_logger(), "No geometry coordinates found in OSRM response.");
                return;
            }
            const Json::Value& coordinates = geometry["coordinates"];
            // Convert coordinates to NavSatFix waypoints
            std::vector<sensor_msgs::msg::NavSatFix> waypoints;
            waypoints.reserve(coordinates.size());

            for (const auto& coord : coordinates) {
                if (!coord.isArray() || coord.size() < 2) {
                    RCLCPP_WARN(this->get_logger(), "Invalid coordinate format.");
                    continue;
                }
                double lat = coord[1].asDouble();
                double lon = coord[0].asDouble();
                sensor_msgs::msg::NavSatFix waypoint;
                waypoint.latitude = lat;
                waypoint.longitude = lon;
                waypoint.altitude = 0.0; // Altitude is not provided by OSRM
                waypoints.push_back(waypoint);
            }
            // Assign waypoints to response
            response->waypoints = waypoints;
            RCLCPP_INFO(this->get_logger(), "Successfully retrieved %zu waypoints.", waypoints.size());
        }
};

int main(int argc, char** argv){
    // Initialize libcurl
    curl_global_init(CURL_GLOBAL_DEFAULT);
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    // Create and spin the node
    auto node = std::make_shared<OsrmRouteServiceNode>();
    rclcpp::spin(node);
    // Shutdown ROS 2
    rclcpp::shutdown();
    // Cleanup libcurl
    curl_global_cleanup();
    return 0;
}
