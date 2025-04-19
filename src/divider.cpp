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

#include "farmbot_interfaces/msg/agent.hpp"
#include "farmbot_interfaces/msg/agents.hpp"
#include "farmbot_interfaces/msg/line.hpp"
#include "farmbot_interfaces/msg/lines.hpp"
#include "farmbot_interfaces/srv/field_op.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "farmbot_interfaces/msg/agent.h"

#include "farmbot_trailblazer/utils/geojson.hpp"
#include <concord/wgs_to_enu.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class FieldOp {
  private:
    rclcpp::Node::SharedPtr node_;
    bool planner_initialized_, got_agent_, got_agents_;
    std::string geojson_file_;
    std::vector<std::string> agent_uuids_;
    std::vector<std::vector<double>> field_points_;
    std::vector<std::vector<double>> geojson_points_;

    farmbot_interfaces::msg::Agent agent_;
    farmbot_interfaces::msg::Agents agents_;

    rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
    rclcpp::Subscription<farmbot_interfaces::msg::Agent>::SharedPtr agent_sub_;
    rclcpp::Subscription<farmbot_interfaces::msg::Agents>::SharedPtr agents_sub_;

    farmbot_interfaces::msg::Lines headlands_msg_, swaths_msg_;
    rclcpp::CallbackGroup::SharedPtr group_one_;
    std::unordered_map<std::string, farmbot_interfaces::msg::Lines> headlands_map_, swaths_map_;

    rclcpp::Service<farmbot_interfaces::srv::FieldOp>::SharedPtr field_gen_service_;

    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr headlands_pub, swaths_pub_;
    rclcpp::TimerBase::SharedPtr self_timer_;

  public:
    farmtrax::Field field;

    FieldOp(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "FIELDOP node started");
        // Callback groups
        group_one_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Create the service
        field_gen_service_ = node_->create_service<farmbot_interfaces::srv::FieldOp>(
            "pln/field_op", std::bind(&FieldOp::field_callback, this, _1, _2), qos_, group_one_);

        headlands_pub = node_->create_publisher<farmbot_interfaces::msg::Lines>("field/headlands", 10);
        swaths_pub_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("field/swaths", 10);

        agent_sub_ = node_->create_subscription<farmbot_interfaces::msg::Agent>(
            "beacon/rci", 10, [this](std::shared_ptr<farmbot_interfaces::msg::Agent> msg) {
                agent_ = *msg;
                got_agent_ = true;
                agent_sub_.reset();
            });
        agents_sub_ = node_->create_subscription<farmbot_interfaces::msg::Agents>(
            "/beacons/rci", 10, [this](std::shared_ptr<farmbot_interfaces::msg::Agents> msg) {
                agents_ = *msg;
                got_agents_ = true;
                agents_sub_.reset();
            });

        self_timer_ = node_->create_wall_timer(1s, [this]() {
            if (!planner_initialized_) {
                return;
            }
            headlands_pub->publish(headlands_msg_);
            swaths_pub_->publish(swaths_msg_);
        });
    }

    void field_callback(std::shared_ptr<farmbot_interfaces::srv::FieldOp::Request> request,
                        std::shared_ptr<farmbot_interfaces::srv::FieldOp::Response> response) {
        if (request->geojson_file.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No geojson file specified");
        }

        if (!got_agent_ || !got_agents_) {
            return;
        }

        geojson_file_ = request->geojson_file;
        agent_uuids_ = request->agent_uuids;

        RCLCPP_INFO(node_->get_logger(), "Field request %s", geojson_file_.c_str());
        auto field = trailblazer::utils::field_from_geojson_file(geojson_file_);

        divide_field(field.border, field.swaths);
        response->message = "Success";
        return;
    }

    void divide_field(farmbot_interfaces::msg::Lines border_msg_, farmbot_interfaces::msg::Lines swaths_msg_) {
        std::vector<std::pair<double, double>> field_points;
        for (const auto &point : border_msg_.lines) {
            field_points.emplace_back(std::make_pair(point.loc_line.front().x, point.loc_line.front().y));
        }
        field.gen_border(field_points);
        std::vector<farmtrax::Swath> swath_vec;
        for (const auto &swath : swaths_msg_.lines) {
            if (swath.done) {
                continue;
            }
            swath_vec.push_back(
                farmtrax::create_swath(farmtrax::Point(swath.loc_line.front().x, swath.loc_line.front().y),
                                       farmtrax::Point(swath.loc_line.back().x, swath.loc_line.back().y),
                                       farmtrax::SwathType::LINE, swath.uuid));
        }
        farmbot_interfaces::msg::Agents agents_willing;
        for (const auto &agent : agents_.agents) {
            if (agent_uuids_.empty() ||
                std::find(agent_uuids_.begin(), agent_uuids_.end(), agent.uuid) != agent_uuids_.end()) {
                agents_willing.agents.push_back(agent);
            }
        }
        uint num_agents = agents_willing.agents.size();
        field.gen_field(swath_vec, 3, num_agents);

        RCLCPP_INFO(node_->get_logger(), "headland size: %lu", field.get_headlands().size());
        RCLCPP_INFO(node_->get_logger(), "swath size: %lu", field.get_swaths().size());
        for (uint i = 0; i < agents_.agents.size(); i++) {
            std::string agent_name = agents_.agents[i].name;
            RCLCPP_INFO(node_->get_logger(), "Agent name: %s", agent_name.c_str());
            for (uint j = 0; j < field.get_headlands()[i].polygon.outer().size(); j++) {
                farmbot_interfaces::msg::Line headland_line;
                geometry_msgs::msg::Point loc_p;
                loc_p.x = field.get_headlands()[i].polygon.outer()[j].x();
                loc_p.y = field.get_headlands()[i].polygon.outer()[j].y();
                headland_line.loc_line.push_back(loc_p);
                headlands_map_[agent_name].lines.push_back(headland_line);
            }
            for (uint j = i; j < field.get_swaths().size(); j += num_agents) {
                farmbot_interfaces::msg::Line swath_msg;
                geometry_msgs::msg::Point loc_p1;
                loc_p1.x = field.get_swaths()[j].line.front().x();
                loc_p1.y = field.get_swaths()[j].line.front().y();
                swath_msg.loc_line.push_back(loc_p1);

                geometry_msgs::msg::Point loc_p2;
                loc_p2.x = field.get_swaths()[j].line.back().x();
                loc_p2.y = field.get_swaths()[j].line.back().y();
                swath_msg.loc_line.push_back(loc_p2);

                swath_msg.uuid = field.get_swaths()[j].uuid;
                swath_msg.length = field.get_swaths()[j].length;
                swaths_map_[agent_name].lines.push_back(swath_msg);
            }
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr genlines_node = rclcpp::Node::make_shared("fieldop", options);
    std::shared_ptr<FieldOp> genlines = std::make_shared<FieldOp>(genlines_node);

    try {
        executor.add_node(genlines_node);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
