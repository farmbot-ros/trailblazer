#include "farmbot_interfaces/msg/lines.hpp"
#include <rclcpp/rclcpp.hpp>

class TaskDivider {
  private:
    rclcpp::Node::SharedPtr node;
    bool swarm;
    std::string border_topic_, swaths_topic_;

    rclcpp::Subscription<farmbot_interfaces::msg::Lines>::SharedPtr border_subscriber_;
    rclcpp::Subscription<farmbot_interfaces::msg::Lines>::SharedPtr swaths_subscriber_;

  public:
    ~TaskDivider() {}
    TaskDivider(rclcpp::Node::SharedPtr node) : node(node) {
        swarm = node->get_parameter_or<bool>("swarm", false);
        if (swarm) {
            border_topic_ = "/field/border";
            swaths_topic_ = "/field/swaths";
        } else {
            border_topic_ = "field/border";
            swaths_topic_ = "field/swaths";
        }
        border_subscriber_ = node->create_subscription<farmbot_interfaces::msg::Lines>(
            border_topic_, 10, std::bind(&TaskDivider::border_callback, this, std::placeholders::_1));
        swaths_subscriber_ = node->create_subscription<farmbot_interfaces::msg::Lines>(
            swaths_topic_, 10, std::bind(&TaskDivider::swaths_callback, this, std::placeholders::_1));
    }

  private:
    void border_callback(const farmbot_interfaces::msg::Lines::SharedPtr msg) {
        RCLCPP_INFO(node->get_logger(), "border callback");
        return;
    }

    void swaths_callback(const farmbot_interfaces::msg::Lines::SharedPtr msg) {
        RCLCPP_INFO(node->get_logger(), "swaths callback");
        return;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr divide_node = rclcpp::Node::make_shared("divide", options);
    std::shared_ptr<TaskDivider> divide = std::make_shared<TaskDivider>(divide_node);

    try {
        executor.add_node(divide_node);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
