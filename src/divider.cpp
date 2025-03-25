#include <rclcpp/rclcpp.hpp>

#include "farmbot_interfaces/msg/agents.hpp"
#include "farmbot_interfaces/msg/lines.hpp"
#include "farmbot_interfaces/srv/field.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class Divider {
  private:
    rclcpp::Node::SharedPtr node_;
    bool recieved_field_, agents_list_received_;

    farmbot_interfaces::msg::Agents agents_list_;
    farmbot_interfaces::msg::Lines border_msg_, swaths_msg_;

    std::vector<std::pair<std::string, rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr>> all_border_pubs_;
    std::vector<std::pair<std::string, rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr>> all_swaths_pubs_;
    std::vector<std::pair<std::string, rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr>>
        all_headland_pubs_;

    rclcpp::CallbackGroup::SharedPtr group_one_, group_two_;
    rclcpp::Service<farmbot_interfaces::srv::Field>::SharedPtr field_service_;
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));

    rclcpp::Subscription<farmbot_interfaces::msg::Agents>::SharedPtr agents_sub_;
    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr border_pub_, swaths_pub_;
    rclcpp::TimerBase::SharedPtr field_timer_, divider_timer_;

  public:
    Divider(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "Divider node started");

        field_service_ = node_->create_service<farmbot_interfaces::srv::Field>(
            "pln/field", std::bind(&Divider::field_callback, this, _1, _2), qos, group_one_);

        field_timer_ = node_->create_wall_timer(1s, std::bind(&Divider::field_timer_callback, this));
        agents_sub_ = node_->create_subscription<farmbot_interfaces::msg::Agents>(
            "/beacons/rci", qos, std::bind(&Divider::agents_callback, this, _1));

        border_pub_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/border", 10);
        swaths_pub_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/swaths", 10);

        divider_timer_ = node_->create_wall_timer(1s, std::bind(&Divider::divider_timer_callback, this));
    }

  private:
    void agents_callback(std::shared_ptr<farmbot_interfaces::msg::Agents> msg) {
        agents_list_ = *msg;
        for (auto agent : agents_list_.beacons) {
            rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr border_pub =
                node_->create_publisher<farmbot_interfaces::msg::Lines>("/" + agent.name + "/pln/border", 10);
            rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr swaths_pub =
                node_->create_publisher<farmbot_interfaces::msg::Lines>("/" + agent.name + "/pln/swaths", 10);
            rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr headland_pub =
                node_->create_publisher<farmbot_interfaces::msg::Lines>("/" + agent.name + "/pln/headland", 10);
            all_border_pubs_.push_back(std::make_pair(agent.name, border_pub));
            all_swaths_pubs_.push_back(std::make_pair(agent.name, swaths_pub));
            all_headland_pubs_.push_back(std::make_pair(agent.name, headland_pub));
        }
        agents_list_received_ = true;
        RCLCPP_INFO(node_->get_logger(), "Agents list received: %lu", agents_list_.beacons.size());
        divide_field();
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
        if (!agents_list_received_) {
            return;
        }
        for (auto agent : agents_list_.beacons) {
            std::string agent_name = agent.name;
            RCLCPP_INFO(node_->get_logger(), "Publishing to %s", agent_name.c_str());
            // auto border_pub = all_border_pubs_[agent_name];
            // auto swaths_pub = all_swaths_pubs_[agent_name];
            // border_pub->publish(border_msg_);
            // swaths_pub->publish(swaths_msg_);
        }
    }

    void divide_field() {
        RCLCPP_INFO(node_->get_logger(), "Dividing field");
        return;
    }
    void field_callback(std::shared_ptr<farmbot_interfaces::srv::Field::Request> request,
                        std::shared_ptr<farmbot_interfaces::srv::Field::Response> response) {

        RCLCPP_INFO(node_->get_logger(), "Swaths received: %lu", request->swaths.lines.size());
        RCLCPP_INFO(node_->get_logger(), "Border received: %lu", request->border.lines.size());
        RCLCPP_INFO(node_->get_logger(), "Agents received: %lu", request->agents.size());
        border_msg_ = request->border;
        swaths_msg_ = request->swaths;
        response->message = "Success";
        recieved_field_ = true;
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
