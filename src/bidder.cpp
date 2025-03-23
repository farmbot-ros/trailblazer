#include "farmbot_interfaces/msg/agent.hpp"
#include "farmbot_interfaces/msg/agents.hpp"
#include "farmbot_interfaces/msg/auction.hpp"
#include "farmbot_interfaces/msg/bid.hpp"
#include "farmbot_interfaces/msg/job.hpp"
#include <rclcpp/rclcpp.hpp>

class Bidder {
  private:
    rclcpp::Node::SharedPtr node;
    std::string namespace_;

    rclcpp::Subscription<farmbot_interfaces::msg::Auction>::SharedPtr auction_subscriber_;
    rclcpp::Publisher<farmbot_interfaces::msg::Bid>::SharedPtr bid_publisher_;

  public:
    ~Bidder() {}
    Bidder(rclcpp::Node::SharedPtr node) : node(node) {
        namespace_ = node->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }
        RCLCPP_INFO(node->get_logger(), "Bidder [%s] started", namespace_.c_str());

        auction_subscriber_ = node->create_subscription<farmbot_interfaces::msg::Auction>(
            "/job/auction", 10, std::bind(&Bidder::auction_callback, this, std::placeholders::_1));
        bid_publisher_ = node->create_publisher<farmbot_interfaces::msg::Bid>("/job/bid", 10);
    }

    void auction_callback(const farmbot_interfaces::msg::Auction::SharedPtr msg) {
        if (msg->job_type != "harvest") {
            return;
        }
        RCLCPP_INFO(node->get_logger(), "Auction with id [%s] received", msg->auction_id.c_str());
        auto key_value = msg->parameters;
        for (const auto &kv : key_value) {
            RCLCPP_INFO(node->get_logger(), "%s: %s", kv.key.c_str(), kv.value.c_str());
        }
    }

  private:
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr divide_node = rclcpp::Node::make_shared("bidder", options);
    std::shared_ptr<Bidder> divide = std::make_shared<Bidder>(divide_node);

    try {
        executor.add_node(divide_node);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
