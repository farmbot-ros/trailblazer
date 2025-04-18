#include <rclcpp/rclcpp.hpp>

#include "farmbot_interfaces/msg/agents.hpp"
#include "farmbot_interfaces/msg/lines.hpp"
#include "farmbot_interfaces/srv/field_op.hpp"

#include <unordered_map>

#include "farmtrax/field.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class Divider {
  private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    bool recieved_field_, agents_list_received_, field_divided_, recieved_self_headland_, recieved_self_swath_;

    farmbot_interfaces::msg::Agents agents_list_;
    farmbot_interfaces::msg::Lines border_msg_, swaths_msg_;

    farmtrax::Field field;

    std::unordered_map<std::string, rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr> swaths_pub_map_;
    std::unordered_map<std::string, farmbot_interfaces::msg::Lines> swaths_map_;
    std::unordered_map<std::string, rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr> headland_pub_map_;
    std::unordered_map<std::string, farmbot_interfaces::msg::Lines> headlands_map_;

    rclcpp::CallbackGroup::SharedPtr group_one_, group_two_;
    rclcpp::Service<farmbot_interfaces::srv::FieldOp>::SharedPtr field_service_;
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));

    rclcpp::Subscription<farmbot_interfaces::msg::Agents>::SharedPtr agents_sub_;
    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr border_pub_, swaths_pub_;
    rclcpp::TimerBase::SharedPtr field_timer_, divider_timer_;

    rclcpp::Subscription<farmbot_interfaces::msg::Lines>::SharedPtr self_headland_sub_, self_swath_sub_;
    farmbot_interfaces::msg::Lines self_headland_msg_, self_swath_msg_;
    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr self_headland_pub_, self_swath_pub_;
    rclcpp::TimerBase::SharedPtr self_timer_;

  public:
    Divider(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "Divider node started");
        namespace_ = node->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }

        field_service_ = node_->create_service<farmbot_interfaces::srv::FieldOp>(
            "pln/field_op", std::bind(&Divider::field_callback, this, _1, _2), qos, group_one_);

        field_timer_ = node_->create_wall_timer(1s, std::bind(&Divider::field_timer_callback, this));
        agents_sub_ = node_->create_subscription<farmbot_interfaces::msg::Agents>(
            "/beacons/rci", qos, std::bind(&Divider::agents_callback, this, _1));

        border_pub_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/border", 10);
        swaths_pub_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/swaths", 10);

        divider_timer_ = node_->create_wall_timer(1s, std::bind(&Divider::divider_timer_callback, this));

        self_headland_sub_ = node_->create_subscription<farmbot_interfaces::msg::Lines>(
            "pln/headland", 10, std::bind(&Divider::self_headland_callback, this, _1));
        self_swath_sub_ = node_->create_subscription<farmbot_interfaces::msg::Lines>(
            "pln/swaths", 10, std::bind(&Divider::self_swath_callback, this, _1));

        self_headland_pub_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("pln/headland", 10);
        self_swath_pub_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("pln/swaths", 10);

        self_timer_ = node_->create_wall_timer(1s, std::bind(&Divider::self_timer_callback, this));
    }

  private:
    void agents_callback(std::shared_ptr<farmbot_interfaces::msg::Agents> msg) {
        for (auto agent : msg->agents) {
            for (auto function : agent.participants[0].functions) {
                if (function != "harvester") {
                    continue;
                }
            }
            agents_list_.agents.push_back(agent);
        }
        for (auto agent : agents_list_.agents) {
            rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr swaths_pub =
                node_->create_publisher<farmbot_interfaces::msg::Lines>("/" + agent.name + "/pln/swaths", 10);
            rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr headland_pub =
                node_->create_publisher<farmbot_interfaces::msg::Lines>("/" + agent.name + "/pln/headland", 10);
            swaths_pub_map_[agent.name] = swaths_pub;
            headland_pub_map_[agent.name] = headland_pub;
        }
        agents_list_received_ = true;
        RCLCPP_INFO(node_->get_logger(), "Agents list received: %lu", agents_list_.agents.size());
        agents_sub_.reset();
    }

    void field_timer_callback() {
        if (!recieved_field_) {
            return;
        }
        border_pub_->publish(border_msg_);
        swaths_pub_->publish(swaths_msg_);
    }

    void divider_timer_callback() {
        if (!agents_list_received_ || !recieved_field_) return;
        if (!field_divided_) {
            divide_field();
            field_divided_ = true;
        }
        for (auto agent : agents_list_.agents) {
            std::string agent_name = agent.name;
            RCLCPP_INFO_ONCE(node_->get_logger(), "Publishing to %s", agent_name.c_str());
            swaths_pub_map_.at(agent_name)->publish(swaths_map_[agent_name]);
            headland_pub_map_.at(agent_name)->publish(headlands_map_[agent_name]);
        }
    }

    void field_callback(std::shared_ptr<farmbot_interfaces::srv::FieldOp::Request> request,
                        std::shared_ptr<farmbot_interfaces::srv::FieldOp::Response> response) {

        RCLCPP_INFO(node_->get_logger(), "Swaths received: %lu", request->field.swaths.lines.size());
        RCLCPP_INFO(node_->get_logger(), "Border received: %lu", request->field.border.lines.size());
        RCLCPP_INFO(node_->get_logger(), "Agents received: %lu", request->agents.size());
        border_msg_ = request->field.border;
        swaths_msg_ = request->field.swaths;
        response->message = "Success";
        recieved_field_ = true;
        return;
    }

    void divide_field() {
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
        uint num_agents = agents_list_.agents.size();
        field.gen_field(swath_vec, 3, num_agents);

        RCLCPP_INFO(node_->get_logger(), "headland size: %lu", field.get_headlands().size());
        RCLCPP_INFO(node_->get_logger(), "swath size: %lu", field.get_swaths().size());
        for (uint i = 0; i < agents_list_.agents.size(); i++) {
            std::string agent_name = agents_list_.agents[i].name;
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
        // RCLCPP_INFO(node_->get_logger(), "---------------- Field divided ----------------");
    }

    void self_headland_callback(std::shared_ptr<farmbot_interfaces::msg::Lines> msg) {
        if (recieved_self_headland_) {
            return;
        }
        self_headland_msg_ = *msg;
        recieved_self_headland_ = true;
        self_headland_sub_.reset();
    }

    void self_swath_callback(std::shared_ptr<farmbot_interfaces::msg::Lines> msg) {
        if (recieved_self_swath_) {
            return;
        }
        self_swath_msg_ = *msg;
        recieved_self_swath_ = true;
        self_swath_sub_.reset();
    }

    void self_timer_callback() {
        if (!recieved_self_headland_ || !recieved_self_swath_) {
            return;
        }
        // RCLCPP_INFO_ONCE(node_->get_logger(), "----------------- Headland and swath received ----------------");
        self_headland_pub_->publish(self_headland_msg_);
        self_swath_pub_->publish(self_swath_msg_);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr divider_node = rclcpp::Node::make_shared("divider", options);
    std::shared_ptr<Divider> divider = std::make_shared<Divider>(divider_node);

    try {
        executor.add_node(divider_node);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
