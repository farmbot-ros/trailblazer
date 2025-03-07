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

#include "farmbot_trailblazer/gen_field.hpp"
#include "farmbot_trailblazer/gen_lines.hpp"

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
    std::shared_ptr<trailblazer::GenField> gen_field_;
    std::shared_ptr<trailblazer::GenLines> gen_lines_;
    double vehicle_coverage_;
    int alternate_freq_;
    double path_angle_;

    // timer
    rclcpp::TimerBase::SharedPtr gen_field_timer_;
    rclcpp::TimerBase::SharedPtr gen_lines_timer_;

  public:
    Generator(rclcpp::Node::SharedPtr node, std::shared_ptr<trailblazer::GenField> gen_field,
              std::shared_ptr<trailblazer::GenLines> gen_lines)
        : node_(node), gen_field_(gen_field), gen_lines_(gen_lines) {

        vehicle_coverage_ = node_->get_parameter_or<double>("vehicle_coverage", 3.0);
        alternate_freq_ = node_->get_parameter_or<int>("alternate_freq", 1);
        path_angle_ = node_->get_parameter_or<double>("path_angle", 90);

        gen_field_timer_ = node_->create_wall_timer(100ms, std::bind(&Generator::gen_field_timer_callback, this));
        gen_lines_timer_ = node_->create_wall_timer(100ms, std::bind(&Generator::gen_lines_timer_callback, this));
    }

    void gen_field_timer_callback() {
        gen_field_->gen_field();
        gen_field_timer_->cancel();
    }

    void gen_lines_timer_callback() {
        if (gen_field_->get_field().empty()) {
            return;
        }
        gen_lines_->gen_lines(gen_field_->get_field());
        gen_lines_timer_->cancel();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr gen_field_node = rclcpp::Node::make_shared("gen_field", options);
    std::shared_ptr<trailblazer::GenField> gen_field = std::make_shared<trailblazer::GenField>(gen_field_node);

    rclcpp::Node::SharedPtr gen_lines_node = rclcpp::Node::make_shared("gen_lines", options);
    std::shared_ptr<trailblazer::GenLines> gen_lines = std::make_shared<trailblazer::GenLines>(gen_field_node);

    rclcpp::Node::SharedPtr generator_node = rclcpp::Node::make_shared("generator", options);
    std::shared_ptr<Generator> generator = std::make_shared<Generator>(generator_node, gen_field, gen_lines);
    try {
        executor.add_node(gen_field_node);
        executor.add_node(gen_lines_node);
        executor.add_node(generator_node);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
