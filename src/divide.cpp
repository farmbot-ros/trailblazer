#include "farmbot_interfaces/msg/lines.hpp"
#include <rclcpp/rclcpp.hpp>

class TaskDivider {
  private:
    rclcpp::Node::SharedPtr node;
    bool swarm;

    rclcpp::Subscription<farmbot_interfaces::msg::Lines>::SharedPtr border_subscriber_;
    rclcpp::Subscription<farmbot_interfaces::msg::Lines>::SharedPtr swaths_subscriber_;

  public:
    TaskDivider(rclcpp::Node::SharedPtr node) : node(node) {}
    // destructor
    ~TaskDivider() {}
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
