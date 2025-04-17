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

#include "farmbot_interfaces/msg/agent.h"

#include <concord/wgs_to_enu.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class GenLines {
  private:
    rclcpp::Node::SharedPtr node_;
    double vehicle_coverage_ = 3.0;
    double path_angle_ = 90;
    bool planner_initialized_, got_agent_;
    std::string geojson_file_;
    std::vector<std::vector<double>> field_points_;
    std::vector<std::vector<double>> geojson_points_;

    farmbot_interfaces::msg::Agent agent_;

    rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));

    farmbot_interfaces::msg::Lines border_msg_, swaths_msg_;
    rclcpp::CallbackGroup::SharedPtr group_one_, group_two_;

    rclcpp::Service<farmbot_interfaces::srv::FieldGen>::SharedPtr field_gen_service_;
    rclcpp::Client<farmbot_interfaces::srv::Field>::SharedPtr field_client_;

    rclcpp::Subscription<farmbot_interfaces::msg::Agent>::SharedPtr agent_sub_;

  public:
    farmtrax::Field field_;

    GenLines(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "GENLINES node started");
        // Callback groups
        group_two_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        group_one_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // Create the service
        field_gen_service_ = node_->create_service<farmbot_interfaces::srv::FieldGen>(
            "pln/field_gen", std::bind(&GenLines::field_callback, this, _1, _2), qos_, group_one_);

        field_client_ = node_->create_client<farmbot_interfaces::srv::Field>("pln/field_msgs");

        agent_sub_ = node_->create_subscription<farmbot_interfaces::msg::Agent>(
            "beacon/rci", 10, std::bind(&GenLines::agent_callback, this, _1));
    }

    void agent_callback(std::shared_ptr<farmbot_interfaces::msg::Agent> msg) {
        agent_ = *msg;
        got_agent_ = true;
    }

    void field_callback(std::shared_ptr<farmbot_interfaces::srv::FieldGen::Request> request,
                        std::shared_ptr<farmbot_interfaces::srv::FieldGen::Response> response) {
        if (request->geojson_file.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No geojson file specified");
            return;
        }

        if (!got_agent_) {
            return;
        }

        geojson_file_ = request->geojson_file;
        vehicle_coverage_ = request->vehicle_coverage;
        path_angle_ = request->path_angle;

        RCLCPP_INFO(node_->get_logger(), "Generating field with %f vehicle coverage and %f path angle",
                    vehicle_coverage_, path_angle_);
        gen_field();
        response->message = "Success";
        response->field.border = border_msg_;
        response->field.swaths = swaths_msg_;
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
        field_.gen_border(field_points_);
        field_.gen_field(vehicle_coverage_, path_angle_);
        auto swaths = field_.get_swaths();
        fill_swaths_msg(swaths);
        RCLCPP_INFO(node_->get_logger(), "Lines generated: %lu", field_.get_swaths().size());
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
        for (const auto &point : navpts) {
            auto enu_point = concord::gps_to_enu(point[0], point[1], point[2], agent_.zero_ref.x, agent_.zero_ref.y,
                                                 agent_.zero_ref.z);
            points_.push_back(
                {std::get<0>(enu_point), std::get<1>(enu_point), std::get<2>(enu_point), point[0], point[1], point[2]});
        }
        return points_;
    }

    std::vector<std::vector<double>> enu_to_nav(const std::vector<std::vector<double>> &points) {
        std::vector<std::vector<double>> navpts_;
        for (const auto &point : points) {
            auto gps_point = concord::enu_to_gps(point[0], point[1], point[2], agent_.zero_ref.x, agent_.zero_ref.y,
                                                 agent_.zero_ref.z);
            navpts_.push_back(
                {std::get<0>(gps_point), std::get<1>(gps_point), std::get<2>(gps_point), point[3], point[4], point[5]});
        }
        return navpts_;
    }

    void fill_border_msg(std::vector<std::vector<double>> points) {
        border_msg_.lines.clear();
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
