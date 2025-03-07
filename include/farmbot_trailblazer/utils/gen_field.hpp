#include "farmbot_trailblazer/farmtrax/field.hpp"
#include "farmbot_trailblazer/farmtrax/mesh.hpp"
#include "farmbot_trailblazer/farmtrax/plan.hpp"
#include "farmbot_trailblazer/farmtrax/route.hpp"
#include "farmbot_trailblazer/farmtrax/swath.hpp"
#include "farmbot_trailblazer/utils/geojson.hpp"
#include <chrono>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/logging.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "farmbot_interfaces/msg/line.hpp"
#include "farmbot_interfaces/msg/lines.hpp"
#include "farmbot_interfaces/srv/field.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// namespace echo = spdlog;

namespace trailblazer {
    using namespace std::chrono_literals;
    using namespace std::placeholders;
    class GenField {
      private:
        double vehicle_coverage_;
        int alternate_freq_;
        double path_angle_;

        bool planner_initialized_ = false;

        std::string namespace_;
        rclcpp::Node::SharedPtr node_;

        farmtrax::Field field_;
        farmtrax::Swaths swaths_;
        farmtrax::Plan plan_;

        farmbot_interfaces::msg::Lines border_msg_;
        farmbot_interfaces::msg::Lines headlands_msg_;
        farmbot_interfaces::msg::Lines swaths_msg_;

        rclcpp::TimerBase::SharedPtr planner_timer_;
        rclcpp::Client<farmbot_interfaces::srv::Field>::SharedPtr get_the_field_client_;

        rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr swaths_publisher_;
        rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr headlands_publisher_;
        rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr border_publisher_;

      public:
        GenField(rclcpp::Node::SharedPtr node) : node_(node) {

            vehicle_coverage_ = node_->get_parameter_or<double>("vehicle_coverage", 3.0);
            // Alternate frequency is the number of robots in the swath
            alternate_freq_ = node_->get_parameter_or<int>("alternate_freq", 1);
            path_angle_ = node_->get_parameter_or<double>("path_angle", 90);
            RCLCPP_INFO(node_->get_logger(), "swath frequency: %i", alternate_freq_);

            // Create the service clients
            get_the_field_client_ = node_->create_client<farmbot_interfaces::srv::Field>("/field/get_field");

            // Timers
            planner_timer_ = node_->create_wall_timer(1s, std::bind(&GenField::timer_callback, this));

            // Line publisher
            border_publisher_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/border", 10);
            swaths_publisher_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/swaths", 10);
            headlands_publisher_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/headlands", 10);

            // Namespace
            namespace_ = node_->get_namespace();
            if (!namespace_.empty() && namespace_[0] == '/') {
                namespace_ = namespace_.substr(1);
            }

            // swaths_.pass_node(node_);
            // plan_.pass_node(node_);
            // field_.pass_node(node_);
        }

      private:
        void timer_callback() {
            if (!planner_initialized_) {
                return;
            }
            border_publisher_->publish(border_msg_);
            swaths_publisher_->publish(swaths_msg_);
            headlands_publisher_->publish(headlands_msg_);
        }

        void gen_swaths(std::vector<std::vector<double>> points) {
            if (points.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get the field");
                return;
            }

            // field_.gen_field(points);
            // border_msg_ = vec_polygon(field_.get_border_points());
            // RCLCPP_INFO(this->get_logger(), "Field generated: %lu", field_.get_border_points().size());
            //
            // swaths_.gen_swaths(field_, vehicle_coverage_, path_angle_, alternate_freq_);
            //
            // swaths_.reverse_swaths();
            // RCLCPP_INFO(this->get_logger(), "Lines generated: %lu", swaths_.get_swaths().size());
            // // headlands_msg_ = vec_polygon_array(swaths_.get_heardlands());
            // plan_.plan_out(swaths_.get_swaths(), alternate_freq_, false);
            // RCLCPP_INFO(this->get_logger(), "Plan generated for %i robots", alternate_freq_);
            //
            // for (unsigned long i = 0; i < plan_.get_swaths_vec().size(); i++) {
            //     auto temp_swath_msg = swaths_to_msg(plan_.get_swaths_vec()[i], "robot" + std::to_string(i));
            //     swaths_msg_.lines.insert(swaths_msg_.lines.end(), temp_swath_msg.lines.begin(),
            //     temp_swath_msg.lines.end());
            // }
            //
            // std::vector<farmtrax::Swath> flat_swaths;
            // for (const auto &swath : plan_.get_swaths_vec()) {
            //     flat_swaths.insert(flat_swaths.end(), swath.begin(), swath.end());
            // }
            // planner_initialized_ = true;
        }

        // std::vector<std::vector<double>> get_field(std::string geojson_file_path = "") {
        //     std::vector<std::vector<double>> points;
        //     auto request = std::make_shared<farmbot_interfaces::srv::Field::Request>();
        //     request->geojson_file = geojson_file_path;
        //     while (!get_the_field_client_->wait_for_service(1s)) {
        //         if (!rclcpp::ok()) {
        //             RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        //             return {};
        //         }
        //         RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        //     }
        //     auto result = get_the_field_client_->async_send_request(request);
        //     auto lpts = result.get()->loc_points;
        //     auto gpts = result.get()->geo_points;
        //     for (int i = 0; i < result.get()->points; i++) {
        //         points.emplace_back(lpts[i].x, lpts[i].y, lpts[i].z, gpts[i].latitude, gpts[i].longitude,
        //         gpts[i].altitude);
        //     }
        //     return points;
        // }
        //
        // farmbot_interfaces::msg::Lines swaths_to_msg(const std::vector<farmtrax::Swath> &swaths, std::string robot) {
        //     farmbot_interfaces::msg::Lines swaths_msg;
        //     for (const auto &swath : swaths) {
        //         farmbot_interfaces::msg::Line swath_msg;
        //         std::vector<geometry_msgs::msg::Point> loc_line;
        //         for (const auto &point : swath.swath) {
        //             geometry_msgs::msg::Point p;
        //             p.x = point.x();
        //             p.y = point.y();
        //             loc_line.push_back(p);
        //         }
        //         swath_msg.loc_line = loc_line;
        //         swath_msg.robot = robot;
        //         swath_msg.length = swath.length;
        //         swath_msg.uuid = swath.uuid;
        //         swath_msg.type = static_cast<uint8_t>(swath.type);
        //         swaths_msg.lines.push_back(swath_msg);
        //     }
        //     return swaths_msg;
        // }
    };
} // namespace trailblazer
