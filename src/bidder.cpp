#include "farmbot_interfaces/msg/agent.hpp"
#include "farmbot_interfaces/msg/agents.hpp"
#include "farmbot_interfaces/msg/auction.hpp"
#include "farmbot_interfaces/msg/bid.hpp"
#include "farmbot_interfaces/msg/job.hpp"
#include "farmbot_interfaces/srv/field.hpp"
#include <cstdlib> // for rand() and srand()
#include <ctime>   // for time()
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class Bidder {
  private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    bool recieved_beacon_;
    farmbot_interfaces::msg::Agent my_beacon_;
    int rand_nr;

    rclcpp::Subscription<farmbot_interfaces::msg::Agent>::SharedPtr beacon_sub_;
    rclcpp::Subscription<farmbot_interfaces::msg::Auction>::SharedPtr auction_sub_;
    rclcpp::Publisher<farmbot_interfaces::msg::Bid>::SharedPtr bid_pub_;
    rclcpp::Subscription<farmbot_interfaces::msg::Job>::SharedPtr job_sub_;
    rclcpp::SubscriptionOptions job_sub_opts_;

    rclcpp::Client<farmbot_interfaces::srv::Field>::SharedPtr field_client_;

  public:
    ~Bidder() {}
    Bidder(rclcpp::Node::SharedPtr node) : node_(node) {
        namespace_ = node->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }
        RCLCPP_INFO(node->get_logger(), "Bidder [%s] started", namespace_.c_str());
        std::srand(std::time(0) + getpid());
        rand_nr = rand() % 100;

        job_sub_opts_.callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        auction_sub_ = node->create_subscription<farmbot_interfaces::msg::Auction>(
            "/job/auction", 10, std::bind(&Bidder::auction_bid, this, _1));
        bid_pub_ = node->create_publisher<farmbot_interfaces::msg::Bid>("/job/bid", 10);
        beacon_sub_ = node->create_subscription<farmbot_interfaces::msg::Agent>(
            "beacon/rci", 10, std::bind(&Bidder::beacon_callback, this, _1));
        job_sub_ = node->create_subscription<farmbot_interfaces::msg::Job>(
            "/job/job", 10, std::bind(&Bidder::job_assignment, this, _1), job_sub_opts_);

        field_client_ = node->create_client<farmbot_interfaces::srv::Field>("pln/field");
    }

  private:
    void beacon_callback(const farmbot_interfaces::msg::Agent::SharedPtr msg) {
        my_beacon_ = *msg;
        RCLCPP_INFO(node_->get_logger(), "Beacon [%s] recieved", my_beacon_.name.c_str());
        recieved_beacon_ = true;
        beacon_sub_.reset();
    }

    void auction_bid(const farmbot_interfaces::msg::Auction::SharedPtr msg) {
        if (msg->job_type != "abliner" || !recieved_beacon_) {
            return;
        }
        RCLCPP_INFO_ONCE(node_->get_logger(), "Auction with id [%s] received", msg->auction_id.c_str());
        auto key_value = msg->parameters;
        // for (const auto &kv : key_value) {//TODO: parse parameters}
        std::string auction_id = msg->auction_id;
        farmbot_interfaces::msg::Bid bid;
        bid.agent = my_beacon_;
        bid.bid = rand_nr;      // TODO: generate bid based on something else than rand_nr
        bid.signature = "test"; // TODO: generate signature
        bid.auction_id = auction_id;
        bid.timestamp = rclcpp::Time(0);
        bid_pub_->publish(bid);
    }

    void job_assignment(const farmbot_interfaces::msg::Job::SharedPtr msg) {
        if (msg->agent.uuid != my_beacon_.uuid) {
            return;
        }
        job_sub_.reset();
        RCLCPP_INFO(node_->get_logger(), "Job [%s] assigned to [%s]", msg->job_id.c_str(), msg->agent.name.c_str());

        auto field_request = std::make_shared<farmbot_interfaces::srv::Field::Request>();
        for (const auto &kv : msg->parameters) {
            if (kv.key == "geojson_file") {
                field_request->geojson_file = kv.value;
                RCLCPP_INFO(node_->get_logger(), "Field request %s", field_request->geojson_file.c_str());
            } else if (kv.key == "vehicle_coverage") {
                field_request->vehicle_coverage = std::stod(kv.value);
                RCLCPP_INFO(node_->get_logger(), "Field request %s",
                            std::to_string(field_request->vehicle_coverage).c_str());
            } else if (kv.key == "path_angle") {
                field_request->path_angle = std::stod(kv.value);
                RCLCPP_INFO(node_->get_logger(), "Field request %s", std::to_string(field_request->path_angle).c_str());
            }
        }
        while (!field_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }
        auto result_future = field_client_->async_send_request(field_request);
        while (rclcpp::ok() && result_future.wait_for(1s) == std::future_status::timeout) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for response from Field service...");
        }
        RCLCPP_INFO(node_->get_logger(), "Successfully recieved Field service response.");
        auto result = result_future.get();
        RCLCPP_INFO(node_->get_logger(), "Recieved %lu swaths", result->swaths.lines.size());
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
