#include "farmbot_interfaces/msg/agent.hpp"
#include "farmbot_interfaces/msg/auction.hpp"
#include "farmbot_interfaces/msg/bid.hpp"
#include "farmbot_interfaces/msg/job.hpp"
#include "farmbot_interfaces/srv/field_gen.hpp"
#include "farmbot_interfaces/srv/field_op.hpp"
#include "farmbot_interfaces/srv/job.hpp"
#include <cstdlib> // for rand() and srand()
#include <ctime>   // for time()
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/bool.hpp>

#include "farmbot_trailblazer/utils/geojson.hpp"
#include <geoson/libgeojson.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

template <typename T> std::vector<uint8_t> serialize(const T &msg) {
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(&msg, &serialized);
    auto rmw_msg = serialized.get_rcl_serialized_message();
    size_t len = rmw_msg.buffer_length;
    auto buf = reinterpret_cast<const uint8_t *>(rmw_msg.buffer);
    std::vector<uint8_t> blob(buf, buf + len);
    return blob;
}

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
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Service<farmbot_interfaces::srv::Job>::SharedPtr job_service_;

    rclcpp::Client<farmbot_interfaces::srv::FieldGen>::SharedPtr field_gen_client_;
    rclcpp::Client<farmbot_interfaces::srv::FieldOp>::SharedPtr field_op_client_;

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

        beacon_sub_ = node->create_subscription<farmbot_interfaces::msg::Agent>(
            "beacon/rci", 10, std::bind(&Bidder::beacon_callback, this, _1));

        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auction_sub_ = node->create_subscription<farmbot_interfaces::msg::Auction>(
            "/job/auction", 10, std::bind(&Bidder::auction_bid, this, _1));
        bid_pub_ = node->create_publisher<farmbot_interfaces::msg::Bid>("/job/bid", 10);
        field_gen_client_ = node->create_client<farmbot_interfaces::srv::FieldGen>("pln/field_gen");
        field_op_client_ = node->create_client<farmbot_interfaces::srv::FieldOp>("pln/field");

        job_service_ = node->create_service<farmbot_interfaces::srv::Job>(
            "job/field_gen", std::bind(&Bidder::job_callback, this, _1, _2), 10, callback_group_);
    }

  private:
    void beacon_callback(const farmbot_interfaces::msg::Agent::SharedPtr msg) {
        my_beacon_ = *msg;
        RCLCPP_INFO(node_->get_logger(), "Beacon [%s] recieved", my_beacon_.name.c_str());
        recieved_beacon_ = true;
        beacon_sub_.reset();
    }

    void auction_bid(const farmbot_interfaces::msg::Auction::SharedPtr msg) {
        if (!recieved_beacon_) {
            return;
        }
        if (msg->job_type == "field_gen") {
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
        } else if (msg->job_type == "field_op") {
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
    }

    void job_callback(std::shared_ptr<farmbot_interfaces::srv::Job::Request> request,
                      std::shared_ptr<farmbot_interfaces::srv::Job::Response> response) {
        RCLCPP_INFO(node_->get_logger(), "Job [%s] assigned to [%s]", request->the_job.job_id.c_str(),
                    request->the_job.agent.name.c_str());

        auto field_gen_request = std::make_shared<farmbot_interfaces::srv::FieldGen::Request>();
        for (const auto &kv : request->the_job.parameters) {
            if (kv.key == "geojson_file") {
                field_gen_request->geojson_file = kv.value;
                RCLCPP_INFO(node_->get_logger(), "Field request %s", field_gen_request->geojson_file.c_str());
            } else if (kv.key == "vehicle_coverage") {
                field_gen_request->vehicle_coverage = std::stod(kv.value);
                RCLCPP_INFO(node_->get_logger(), "Field request %s",
                            std::to_string(field_gen_request->vehicle_coverage).c_str());
            } else if (kv.key == "path_angle") {
                field_gen_request->path_angle = std::stod(kv.value);
                RCLCPP_INFO(node_->get_logger(), "Field request %s",
                            std::to_string(field_gen_request->path_angle).c_str());
            }
        }
        while (!field_gen_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }
        auto fg_future = field_gen_client_->async_send_request(field_gen_request);
        while (rclcpp::ok() && fg_future.wait_for(1s) == std::future_status::timeout) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for response from Field service...");
        }
        auto fg_result = fg_future.get();
        RCLCPP_INFO(node_->get_logger(), "Successfully recieved FieldGen service response.");

        RCLCPP_INFO(node_->get_logger(), "Border received: %lu", fg_result->field.border.lines.size());
        RCLCPP_INFO(node_->get_logger(), "Swaths received: %lu", fg_result->field.swaths.lines.size());

        nlohmann::json gsn = trailblazer::utils::colleciton_from_field(fg_result->field);
        // std::cout << gsn.dump(4) << std::endl;
        // geoson::op::SaveFeatureCollection("/tmp/field_g.geojson", gsn);

        // ------------------- Response -------------------
        response->message = "Success";
        response->type = "json/Field";
        response->data = nlohmann::json::to_cbor(gsn);
        // response->data = serialize(fg_result->field);
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
