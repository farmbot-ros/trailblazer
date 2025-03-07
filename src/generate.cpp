#include "farmbot_trailblazer/farmtrax/field.hpp"
#include "farmbot_trailblazer/farmtrax/plan.hpp"
#include "farmbot_trailblazer/farmtrax/swath.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "farmbot_trailblazer/utils/gen_field.hpp"
#include "farmbot_trailblazer/utils/get_field.hpp"

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
using namespace std::chrono_literals;
using namespace std::placeholders;

class Generator {
  private:
    rclcpp::Node::SharedPtr node_;
    double vehicle_coverage_;
    int alternate_freq_;
    double path_angle_;

    trailblazer::GenField gen_field_;
    trailblazer::GetField get_field_;

  public:
    Generator(rclcpp::Node::SharedPtr node) : node_(node) {
        // gen_field_.init(node);
        get_field_.init(node);

        vehicle_coverage_ = node_->get_parameter_or<double>("vehicle_coverage", 3.0);
        // Alternate frequency is the number of robots in the swath
        alternate_freq_ = node_->get_parameter_or<int>("alternate_freq", 1);
        path_angle_ = node_->get_parameter_or<double>("path_angle", 90);
        RCLCPP_INFO(node_->get_logger(), "swath frequency: %i", alternate_freq_);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("gen_lines", options);
    Generator generator(node);

    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Could not spin executor: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
