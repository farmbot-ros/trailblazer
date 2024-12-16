#include <memory>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "farmbot_interfaces/srv/go_to_field.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "farmbot_interfaces/srv/enu2_gps.hpp"

#include <curl/curl.h>
#include <json/json.h> // Install jsoncpp or use another JSON library

using namespace std::chrono_literals;
using namespace std::placeholders;
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
        rclcpp::Service<GetRoute>::SharedPtr service_;
        std::string osrm_server_url_;
        rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
        rclcpp::Client<farmbot_interfaces::srv::Enu2Gps>::SharedPtr enu2gps_client_;

        sensor_msgs::msg::NavSatFix robot_loc_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr loc_sub_;
        bool entry_loc_set_ = false;
        sensor_msgs::msg::NavSatFix entry_loc_;
        bool entry_loc_inited_ = false;
        geometry_msgs::msg::Pose entry_loc_enu_;

    public:
        OsrmRouteServiceNode(): Node("goto_field"){
            // Set the public OSRM server URL
            osrm_server_url_ = this->get_parameter_or<std::string>("osrm_server_url", "http://router.project-osrm.org");

            gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>("loc/gps2enu");
            enu2gps_client_ = this->create_client<farmbot_interfaces::srv::Enu2Gps>("loc/enu2gps");

            // Create the service
            service_ = this->create_service<GetRoute>(
                "pln/get_route",
                std::bind(&OsrmRouteServiceNode::handle_get_route, this, std::placeholders::_1, std::placeholders::_2)
            );

            loc_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("loc/fix", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                robot_loc_ = *msg;
            });
            RCLCPP_INFO(this->get_logger(), "OSRM Route Service Node is ready.");

            // run in thread to wait for entry location
            std::thread(&OsrmRouteServiceNode::run_in_thread, this).detach();
        }

    private:

        void run_in_thread(){
            while(rclcpp::ok() && !entry_loc_inited_){
                // RCLCPP_INFO(this->get_logger(), "Waiting for entry location...");
            }
            auto navpts = enu_to_nav({entry_loc_enu_});
            entry_loc_ = navpts[0];
            entry_loc_set_ = true;
        }

        void handle_get_route(const std::shared_ptr<GetRoute::Request> request, std::shared_ptr<GetRoute::Response> response){
            RCLCPP_INFO(this->get_logger(), "Received GetRoute request.");

            entry_loc_enu_ = request->point;
            entry_loc_inited_ = true;

            while(rclcpp::ok() && !entry_loc_set_){
                // RCLCPP_INFO(this->get_logger(), "Waiting for entry location...");
            }

            // Extract start and end coordinates
            double start_lat = robot_loc_.latitude;
            double start_lon = robot_loc_.longitude;
            double end_lat = entry_loc_.latitude;
            double end_lon = entry_loc_.longitude;

            // Construct the OSRM API URL using the public server
            std::ostringstream url_stream;
            url_stream << osrm_server_url_ << "/route/v1/foot/"
                    << start_lon << "," << start_lat << ";"
                    << end_lon << "," << end_lat
                    << "?geometries=geojson&overview=full";
            std::string url = url_stream.str();

            RCLCPP_INFO(this->get_logger(), "OSRM API URL: %s", url.c_str());

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
            response->points = nav_to_enu(waypoints);
            RCLCPP_INFO(this->get_logger(), "Successfully retrieved %zu waypoints.", waypoints.size());
        }

        std::vector<geometry_msgs::msg::Point> nav_to_enu(std::vector<sensor_msgs::msg::NavSatFix> navpts){
            RCLCPP_INFO(this->get_logger(), "Converting NavSatFix to ENU");
            std::vector<geometry_msgs::msg::Point> points;
            auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();
            for (const auto& point : navpts) {
                request->gps.push_back(point);
            }
            while (!gps2enu_client_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return std::vector<geometry_msgs::msg::Point>();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto result = gps2enu_client_->async_send_request(request);
            auto getres =  result.get()->enu;
            for (const auto& point : getres) {
                geometry_msgs::msg::Point p;
                p.x = point.position.x;
                p.y = point.position.y;
                p.z = point.position.z;
                points.push_back(p);
            }
            return points;
        }

        std::vector<sensor_msgs::msg::NavSatFix> enu_to_nav(std::vector<geometry_msgs::msg::Pose> enupts){
            RCLCPP_INFO(this->get_logger(), "Converting ENU to NavSatFix");
            std::vector<sensor_msgs::msg::NavSatFix> points;
            auto request = std::make_shared<farmbot_interfaces::srv::Enu2Gps::Request>();
            for (const auto& point : enupts) {
                request->enu.push_back(point);
            }
            while (!enu2gps_client_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return std::vector<sensor_msgs::msg::NavSatFix>();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto result = enu2gps_client_->async_send_request(request);
            // while (rclcpp::ok() && result.wait_for(1s) == std::future_status::timeout) {
                // RCLCPP_INFO(this->get_logger(), "Waiting for response for ENU to GPS conversion...");
            // }
            auto getres =  result.get()->gps;
            for (const auto& point : getres) {
                sensor_msgs::msg::NavSatFix p;
                p.latitude = point.latitude;
                p.longitude = point.longitude;
                p.altitude = point.altitude;
                points.push_back(p);
            }
            return points;
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
