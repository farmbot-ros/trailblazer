#include "farmbot_interfaces/msg/agent.hpp"
#include "farmbot_interfaces/msg/agents.hpp"
#include "farmbot_interfaces/msg/auction.hpp"
#include "farmbot_interfaces/msg/bid.hpp"
#include "farmbot_interfaces/msg/job.hpp"
#include <cstdlib> // for rand() and srand()
#include <ctime>   // for time()
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class Bidder {
  private:
    rclcpp::Node::SharedPtr node;
    std::string namespace_;
    bool recieved_beacon_;
    farmbot_interfaces::msg::Agent my_beacon_;
    int rand_nr;

    rclcpp::Subscription<farmbot_interfaces::msg::Agent>::SharedPtr beacon_subscriber_;
    rclcpp::Subscription<farmbot_interfaces::msg::Auction>::SharedPtr auction_subscriber_;
    rclcpp::Publisher<farmbot_interfaces::msg::Bid>::SharedPtr bid_publisher_;
    rclcpp::Subscription<farmbot_interfaces::msg::Job>::SharedPtr job_subscriber_;

  public:
    ~Bidder() {}
    Bidder(rclcpp::Node::SharedPtr node) : node(node) {
        namespace_ = node->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }
        RCLCPP_INFO(node->get_logger(), "Bidder [%s] started", namespace_.c_str());
        std::srand(std::time(0) + getpid());
        rand_nr = rand() % 100;

        auction_subscriber_ = node->create_subscription<farmbot_interfaces::msg::Auction>(
            "/job/auction", 10, std::bind(&Bidder::auction_bid, this, std::placeholders::_1));
        bid_publisher_ = node->create_publisher<farmbot_interfaces::msg::Bid>("/job/bid", 10);
        beacon_subscriber_ = node->create_subscription<farmbot_interfaces::msg::Agent>(
            "beacon/rci", 10, std::bind(&Bidder::beacon_callback, this, std::placeholders::_1));
        job_subscriber_ = node->create_subscription<farmbot_interfaces::msg::Job>(
            "/job/job", 10, std::bind(&Bidder::job_assignment, this, std::placeholders::_1));
    }

  private:
    void beacon_callback(const farmbot_interfaces::msg::Agent::SharedPtr msg) {
        my_beacon_ = *msg;
        RCLCPP_INFO(node->get_logger(), "Beacon [%s] recieved", my_beacon_.name.c_str());
        recieved_beacon_ = true;
        beacon_subscriber_.reset();
    }

    void auction_bid(const farmbot_interfaces::msg::Auction::SharedPtr msg) {
        if (msg->job_type != "harvest" || !recieved_beacon_) {
            return;
        }
        RCLCPP_INFO_ONCE(node->get_logger(), "Auction with id [%s] received", msg->auction_id.c_str());
        auto key_value = msg->parameters;
        for (const auto &kv : key_value) {
            RCLCPP_INFO_ONCE(node->get_logger(), "%s: %s", kv.key.c_str(), kv.value.c_str());
        }

        std::string auction_id = msg->auction_id;
        farmbot_interfaces::msg::Bid bid;
        bid.agent = my_beacon_;
        bid.bid = rand_nr;
        bid.signature = "test";
        bid.auction_id = auction_id;
        bid.timestamp = rclcpp::Time(0);
        bid_publisher_->publish(bid);
    }

    void job_assignment(const farmbot_interfaces::msg::Job::SharedPtr msg) {
        if (msg->agent.uuid != my_beacon_.uuid) {
            return;
        }
        RCLCPP_INFO(node->get_logger(), "Job [%s] assigned to [%s]", msg->job_id.c_str(), msg->agent.name.c_str());
    }
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
