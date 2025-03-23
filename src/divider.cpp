#include <rclcpp/rclcpp.hpp>

class Divider {
  private:
    rclcpp::Node::SharedPtr node_;

  public:
    Divider(rclcpp::Node::SharedPtr node) : node_(node) { RCLCPP_INFO(node_->get_logger(), "Divider node started"); }
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
