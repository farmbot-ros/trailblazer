#include <rclcpp/rclcpp.hpp>

#include "farmbot_interfaces/msg/lines.hpp"
#include "farmbot_interfaces/srv/field.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class Divider {
  private:
    rclcpp::Node::SharedPtr node_;
    bool recieved_field_;

    farmbot_interfaces::msg::Lines border_msg_, swaths_msg_;

    rclcpp::CallbackGroup::SharedPtr group_one_, group_two_;
    rclcpp::Service<farmbot_interfaces::srv::Field>::SharedPtr field_service_;

    rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr border_pub_, swaths_pub_;
    rclcpp::TimerBase::SharedPtr field_timer_;

  public:
    Divider(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "Divider node started");

        field_service_ = node_->create_service<farmbot_interfaces::srv::Field>(
            "pln/field", std::bind(&Divider::field_callback, this, _1, _2), rmw_qos_profile_services_default,
            group_one_);

        field_timer_ = node_->create_wall_timer(1s, std::bind(&Divider::field_timer_callback, this));

        border_pub_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/border", 10);
        swaths_pub_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/swaths", 10);
    }

  private:
    void field_timer_callback() {
        if (!recieved_field_) {
            return;
        }
        border_pub_->publish(border_msg_);
        swaths_pub_->publish(swaths_msg_);
    }

  private:
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
