#include <chrono>
#include <json/json.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "farmtrax/field.hpp"
#include "farmtrax/mesh.hpp"
#include "farmtrax/plan.hpp"
#include "farmtrax/route.hpp"
#include "farmtrax/swath.hpp"

#include "farmbot_interfaces/msg/line.hpp"
#include "farmbot_interfaces/msg/lines.hpp"
#include "farmbot_interfaces/srv/field.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "farmbot_trailblazer/utils/geojson.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using Field = farmbot_interfaces::srv::Field;
using namespace std::chrono_literals;
using namespace std::placeholders;

class GenLines {
  private:
    rclcpp::Node::SharedPtr node_;
    int alternate_freq_;
    double vehicle_coverage_, path_angle_;
    bool planner_initialized_;
    std::string geojson_file_;
    std::vector<std::vector<double>> field_points_;
    std::vector<std::vector<double>> geojson_points_;

    farmbot_interfaces::msg::Lines border_msg_;
    farmbot_interfaces::msg::Lines headlands_msg_;
    farmbot_interfaces::msg::Lines swaths_msg_;

    rclcpp::TimerBase::SharedPtr planner_timer_;
    rclcpp::TimerBase::SharedPtr just_timer_;
    rclcpp::Client<farmbot_interfaces::srv::Field>::SharedPtr get_the_field_client_;

    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr swaths_publisher_;
    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr headlands_publisher_;
    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr border_publisher_;

    rclcpp::CallbackGroup::SharedPtr client_group_, service_group_;
    rmw_qos_profile_t qos_profile;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;

  public:
    farmtrax::Field field_;
    farmtrax::Swaths swaths_;
    farmtrax::Plan plan_;

    GenLines(rclcpp::Node::SharedPtr node) : node_(node) {
        vehicle_coverage_ = node_->get_parameter_or<double>("vehicle_coverage", 3.0);
        alternate_freq_ = node_->get_parameter_or<int>("alternate_freq", 1);
        path_angle_ = node_->get_parameter_or<double>("path_angle", 90);
        geojson_file_ = node_->get_parameter_or<std::string>("geojson_file", "field.geojson");
        // Callback groups
        service_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        client_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        qos_profile = rmw_qos_profile_services_default;
        // Create the service clients
        get_the_field_client_ = node_->create_client<farmbot_interfaces::srv::Field>("/field/get_field");
        // Timers
        planner_timer_ = node_->create_wall_timer(1s, std::bind(&GenLines::timer_callback, this));
        just_timer_ = node_->create_wall_timer(1s, std::bind(&GenLines::gen_lines, this), service_group_);
        // Line publisher
        border_publisher_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/border", 10);
        swaths_publisher_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/swaths", 10);
        headlands_publisher_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/headlands", 10);
        // Create the service clients
        gps2enu_client_ =
            node_->create_client<farmbot_interfaces::srv::Gps2Enu>("loc/gps2enu", qos_profile, client_group_);
    }

    void gen_lines() {
        getPointsFromGeoJSON(geojson_file_);
        navToEnu(geojson_points_);

        while (field_points_.empty() && rclcpp::ok()) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for field...");
        }
        std::vector<std::vector<double>> points = field_points_;
        genenerate(points);
        just_timer_->cancel();
    }

  private:
    void timer_callback() {
        if (!planner_initialized_) {
            return;
        }
        border_publisher_->publish(border_msg_);
        // swaths_publisher_->publish(swaths_msg_);
        // headlands_publisher_->publish(headlands_msg_);
    }

    void genenerate(std::vector<std::vector<double>> points) {
        if (points.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get the field");
            return;
        }

        fill_border_msg(points);
        RCLCPP_INFO(node_->get_logger(), "Field generated: %lu", points.size());

        field_ = farmtrax::Field(points);
        swaths_.gen_swaths(field_, vehicle_coverage_, path_angle_, alternate_freq_);
        //
        // swaths_.reverse_swaths();
        RCLCPP_INFO(node_->get_logger(), "Lines generated: %lu", swaths_.get_swaths().size());
        // // headlands_msg_ = vec_polygon_array(swaths_.get_heardlands());
        // plan_.plan_out(swaths_.get_swaths(), alternate_freq_, false);
        // RCLCPP_INFO(this->get_logger(), "Plan generated for %i robots", alternate_freq_);
        //
        // for (unsigned long i = 0; i < plan_.get_swaths_vec().size(); i++) {
        //     auto temp_swath_msg = swaths_to_msg(plan_.get_swaths_vec()[i], "robot" + std::to_string(i));
        //     swaths_msg_.lines.insert(swaths_msg_.lines.end(), temp_swath_msg.lines.begin(),
        //     temp_swath_msg.lines.end());
        // }
        //
        // std::vector<farmtrax::Swath> flat_swaths;
        // for (const auto &swath : plan_.get_swaths_vec()) {
        //     flat_swaths.insert(flat_swaths.end(), swath.begin(), swath.end());
        // }
        planner_initialized_ = true;
    }

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
        auto getres = result->enu;
        for (uint i = 0; i < getres.size(); i++) {
            field_points_.push_back({getres[i].position.x, getres[i].position.y, getres[i].position.z, navpts[i][0],
                                     navpts[i][1], navpts[i][2]});
        }
        RCLCPP_INFO(node_->get_logger(), "Successfully retrieved %zu waypoints.", field_points_.size());
    }

    void fill_border_msg(std::vector<std::vector<double>> points) {
        RCLCPP_INFO(node_->get_logger(), "Border received: %lu", points.size());
        for (const auto &point : points) {
            farmbot_interfaces::msg::Line border_msg;

            geometry_msgs::msg::Point loc_p;
            loc_p.x = point[0];
            loc_p.y = point[1];
            loc_p.z = point[2];
            border_msg.loc_line.push_back(loc_p);

            geometry_msgs::msg::Point geo_p;
            geo_p.x = point[3];
            geo_p.y = point[4];
            geo_p.z = point[5];
            border_msg.geo_line.push_back(geo_p);

            border_msg_.lines.push_back(border_msg);
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr genlines_node = rclcpp::Node::make_shared("genlines", options);
    std::shared_ptr<GenLines> genlines = std::make_shared<GenLines>(genlines_node);

    try {
        executor.add_node(genlines_node);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
