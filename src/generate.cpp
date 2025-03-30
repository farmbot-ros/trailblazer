#include <json/json.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>
#include <string>
#include <vector>

#include "farmtrax/field.hpp"
#include "farmtrax/mesh.hpp"
#include "farmtrax/plan.hpp"

#include "farmbot_interfaces/msg/line.hpp"
#include "farmbot_interfaces/msg/lines.hpp"
#include "farmbot_interfaces/srv/enu2_gps.hpp"
#include "farmbot_interfaces/srv/field.hpp"
#include "farmbot_interfaces/srv/field_gen.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"

#include "farmbot_trailblazer/utils/geojson.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class GenLines {
  private:
    rclcpp::Node::SharedPtr node_;
    double vehicle_coverage_ = 3.0;
    double path_angle_ = 90;
    bool planner_initialized_;
    std::string geojson_file_;
    std::vector<std::vector<double>> field_points_;
    std::vector<std::vector<double>> geojson_points_;

    rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));

    farmbot_interfaces::msg::Lines border_msg_, swaths_msg_;
    rclcpp::CallbackGroup::SharedPtr group_one_, group_two_;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    rclcpp::Client<farmbot_interfaces::srv::Enu2Gps>::SharedPtr enu2gps_client_;
    rclcpp::Service<farmbot_interfaces::srv::FieldGen>::SharedPtr field_gen_service_;
    rclcpp::Client<farmbot_interfaces::srv::Field>::SharedPtr field_client_;

  public:
    farmtrax::Border border_;
    farmtrax::Field swaths_;
    farmtrax::Plan plan_;

    GenLines(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "GENLINES node started");
        // Callback groups
        group_two_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        group_one_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // Create the service clients
        gps2enu_client_ = node_->create_client<farmbot_interfaces::srv::Gps2Enu>("loc/gps2enu");
        enu2gps_client_ = node_->create_client<farmbot_interfaces::srv::Enu2Gps>("loc/enu2gps");
        // Create the service
        field_gen_service_ = node_->create_service<farmbot_interfaces::srv::FieldGen>(
            "pln/field_gen", std::bind(&GenLines::field_callback, this, _1, _2), qos_, group_one_);

        field_client_ = node_->create_client<farmbot_interfaces::srv::Field>("pln/field_msgs");
    }

    void field_callback(std::shared_ptr<farmbot_interfaces::srv::FieldGen::Request> request,
                        std::shared_ptr<farmbot_interfaces::srv::FieldGen::Response> response) {
        if (request->geojson_file.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No geojson file specified");
            return;
        }

        geojson_file_ = request->geojson_file;
        vehicle_coverage_ = request->vehicle_coverage;
        path_angle_ = request->path_angle;

        RCLCPP_INFO(node_->get_logger(), "Generating field with %f vehicle coverage and %f path angle",
                    vehicle_coverage_, path_angle_);

        gen_field();

        response->message = "Success";
        response->border = border_msg_;
        response->swaths = swaths_msg_;
        return;
    }

    void gen_field() {
        points_jsonfile(geojson_file_);
        field_points_ = nav_to_enu(geojson_points_);
        if (field_points_.empty()) {
            return;
        }
        genenerate_swaths();
    }

  private:
    void genenerate_swaths() {
        if (field_points_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get the field");
            return;
        }
        fill_border_msg(field_points_);
        RCLCPP_INFO(node_->get_logger(), "Field generated: %lu", field_points_.size());
        border_ = farmtrax::Border(field_points_);
        swaths_.gen_field(border_, vehicle_coverage_, path_angle_);
        auto swaths = swaths_.get_swaths();
        fill_swaths_msg(swaths);
        RCLCPP_INFO(node_->get_logger(), "Lines generated: %lu", swaths_.get_swaths().size());
        planner_initialized_ = true;
    }

  private:
    void points_jsonfile(const std::string &geojson_file) {
        try {
            auto geojsonObject = geojson::parseGeoJSONFromFile(geojson_file);
            geojson_points_ = geojson::utils::extractFirstPolygon(geojsonObject);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "Error parsing GeoJSON: %s", e.what());
        }
    }

    std::vector<std::vector<double>> nav_to_enu(const std::vector<std::vector<double>> &navpts) {
        std::vector<std::vector<double>> points_;
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
                return points_;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }
        auto result_future = gps2enu_client_->async_send_request(request);
        while (rclcpp::ok() && result_future.wait_for(1s) == std::future_status::timeout) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for response from GPS2ENU service...");
        }
        RCLCPP_INFO(node_->get_logger(), "Successfully recieved GPS2ENU service response.");
        auto result = result_future.get();
        auto getres = result->enu;
        for (uint i = 0; i < getres.size(); i++) {
            points_.push_back({getres[i].position.x, getres[i].position.y, getres[i].position.z, navpts[i][0],
                               navpts[i][1], navpts[i][2]});
        }
        RCLCPP_INFO(node_->get_logger(), "Successfully retrieved %zu waypoints.", points_.size());
        return points_;
    }

    std::vector<std::vector<double>> enu_to_nav(const std::vector<std::vector<double>> &points) {
        std::vector<std::vector<double>> navpts_;
        auto request = std::make_shared<farmbot_interfaces::srv::Enu2Gps::Request>();
        for (const auto &point : points) {
            geometry_msgs::msg::Pose enu_point;
            enu_point.position.x = point[0];
            enu_point.position.y = point[1];
            enu_point.position.z = point[2];
            request->enu.push_back(enu_point);
        }
        while (!enu2gps_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return navpts_;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }
        auto result_future = enu2gps_client_->async_send_request(request);
        while (rclcpp::ok() && result_future.wait_for(1s) == std::future_status::timeout) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for response from ENU2GPS service...");
        }
        RCLCPP_INFO(node_->get_logger(), "Successfully recieved ENU2GPS service response.");
        auto result = result_future.get();
        auto getres = result->gps;
        for (uint i = 0; i < getres.size(); i++) {
            navpts_.push_back({getres[i].latitude, getres[i].longitude, getres[i].altitude, points[i][0], points[i][1],
                               points[i][2]});
        }
        RCLCPP_INFO(node_->get_logger(), "Successfully retrieved %zu waypoints.", navpts_.size());
        return navpts_;
    }

    void fill_border_msg(std::vector<std::vector<double>> points) {
        border_msg_.lines.clear();
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

    void fill_swaths_msg(std::vector<farmtrax::Swath> swaths) {
        swaths_msg_.lines.clear();
        RCLCPP_INFO(node_->get_logger(), "Swaths received: %lu", swaths.size());
        std::vector<std::vector<double>> local_temp;
        for (const auto &swath : swaths) {
            local_temp.push_back({swath.line.front().x(), swath.line.front().y(), .0});
            local_temp.push_back({swath.line.back().x(), swath.line.back().y(), .0});
        }
        auto navs = enu_to_nav(local_temp);
        for (uint i = 0; i < navs.size(); i += 2) {
            farmbot_interfaces::msg::Line swath_msg;
            geometry_msgs::msg::Point geo_p1;
            geo_p1.x = navs[i][0];
            geo_p1.y = navs[i][1];
            geo_p1.z = navs[i][2];
            swath_msg.geo_line.push_back(geo_p1);
            geometry_msgs::msg::Point geo_p2;
            geo_p2.x = navs[i + 1][0];
            geo_p2.y = navs[i + 1][1];
            geo_p2.z = navs[i + 1][2];
            swath_msg.geo_line.push_back(geo_p2);

            geometry_msgs::msg::Point loc_p1;
            loc_p1.x = navs[i][3];
            loc_p1.y = navs[i][4];
            loc_p1.z = navs[i][5];
            swath_msg.loc_line.push_back(loc_p1);
            geometry_msgs::msg::Point loc_p2;
            loc_p2.x = navs[i + 1][3];
            loc_p2.y = navs[i + 1][4];
            loc_p2.z = navs[i + 1][5];
            swath_msg.loc_line.push_back(loc_p2);
            swaths_msg_.lines.push_back(swath_msg);
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
