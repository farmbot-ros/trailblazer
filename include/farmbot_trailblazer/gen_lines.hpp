#include "farmbot_trailblazer/utils/geojson.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/mesh.hpp"
#include "farmtrax/plan.hpp"
#include "farmtrax/route.hpp"
#include "farmtrax/swath.hpp"
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

namespace trailblazer {
    using namespace std::chrono_literals;
    using namespace std::placeholders;
    class GenLines {
      private:
        double vehicle_coverage_;
        int alternate_freq_;
        double path_angle_;
        bool planner_initialized_ = false;

        std::string namespace_;
        rclcpp::Node::SharedPtr node_;

        farmbot_interfaces::msg::Lines border_msg_;
        farmbot_interfaces::msg::Lines headlands_msg_;
        farmbot_interfaces::msg::Lines swaths_msg_;

        rclcpp::TimerBase::SharedPtr planner_timer_;
        rclcpp::Client<farmbot_interfaces::srv::Field>::SharedPtr get_the_field_client_;

        rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr swaths_publisher_;
        rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr headlands_publisher_;
        rclcpp::Publisher<farmbot_interfaces::msg::Lines>::SharedPtr border_publisher_;

      public:
        farmtrax::Field field_;
        farmtrax::Swaths swaths_;
        farmtrax::Plan plan_;

        GenLines(rclcpp::Node::SharedPtr node) : node_(node) {
            vehicle_coverage_ = node_->get_parameter_or<double>("vehicle_coverage", 3.0);
            // Alternate frequency is the number of robots in the swath
            alternate_freq_ = node_->get_parameter_or<int>("alternate_freq", 1);
            path_angle_ = node_->get_parameter_or<double>("path_angle", 90);
            RCLCPP_INFO(node_->get_logger(), "swath frequency: %i", alternate_freq_);
            // Create the service clients
            get_the_field_client_ = node_->create_client<farmbot_interfaces::srv::Field>("/field/get_field");
            // Timers
            planner_timer_ = node_->create_wall_timer(1s, std::bind(&GenLines::timer_callback, this));
            // Line publisher
            border_publisher_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/border", 10);
            swaths_publisher_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/swaths", 10);
            headlands_publisher_ = node_->create_publisher<farmbot_interfaces::msg::Lines>("/field/headlands", 10);
            // Namespace
            namespace_ = node_->get_namespace();
            if (!namespace_.empty() && namespace_[0] == '/') {
                namespace_ = namespace_.substr(1);
            }
        }

        void gen_lines(std::vector<std::vector<double>> points) { genenerate(points); }

      private:
        void timer_callback() {
            if (!planner_initialized_) {
                return;
            }
            border_publisher_->publish(border_msg_);
            // swaths_publisher_->publish(swaths_msg_);
            // headlands_publisher_->publish(headlands_msg_);
        }

        void genenerate(std::vector<std::vector<double>> points) {
            if (points.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get the field");
                return;
            }

            fill_border_msg(points);
            // RCLCPP_INFO(node_->get_logger(), "Field generated: %lu", points.size());

            field_ = farmtrax::Field(points);
            swaths_.gen_swaths(field_, vehicle_coverage_, path_angle_, alternate_freq_);
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
            planner_initialized_ = true;
        }

        void fill_border_msg(std::vector<std::vector<double>> points) {
            RCLCPP_INFO(node_->get_logger(), "Border received: %lu", points.size());
            for (const auto &point : points) {
                farmbot_interfaces::msg::Line border_msg;

                geometry_msgs::msg::Point loc_p;
                loc_p.x = point[0];
                loc_p.y = point[1];
                loc_p.z = point[2];
                border_msg.loc_line.push_back(loc_p);

                geometry_msgs::msg::Point geo_p;
                geo_p.x = point[3];
                geo_p.y = point[4];
                geo_p.z = point[5];
                border_msg.geo_line.push_back(geo_p);

                border_msg_.lines.push_back(border_msg);
            }
        }
    };
} // namespace trailblazer
