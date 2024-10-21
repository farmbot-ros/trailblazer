#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include <iostream>
#include "farmbot_planner/utils/geojson.hpp"
#include "farmbot_planner/farmtrax/field.hpp"
#include "farmbot_planner/farmtrax/swath.hpp"
#include "farmbot_planner/farmtrax/planner.hpp"
#include "farmbot_planner/farmtrax/mesh.hpp"
#include "farmbot_planner/farmtrax/path.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "farmbot_interfaces/srv/enu2_gps.hpp"
#include "farmbot_interfaces/srv/go_to_field.hpp"
#include "farmbot_interfaces/srv/get_the_field.hpp"

#include "farmbot_interfaces/msg/waypoint.hpp"
#include "farmbot_interfaces/msg/segment.hpp"
#include "farmbot_interfaces/msg/segments.hpp"


using namespace std::chrono_literals;

struct PathSegment {
    std::pair<double, double> start;
    std::pair<double, double> end;
    std::vector<std::pair<double, double>> middle;
};

class FieldProcessorNode : public rclcpp::Node {
private:
    std::string name;
    std::string topic_prefix_param;
    double vehicle_width_;
    double vehicle_coverage_;
    int alternate_freq_;
    double path_angle_;
    bool goto_field_;

    bool planner_initialized_ = false;
    bool gps_locked_ = false;

    farmtrax::Field field_;
    farmtrax::Swaths swaths_;
    farmtrax::Planner planner_;

    farmtrax::Mesh mesh_;
    farmtrax::Path pathh_;
    farmtrax::Swaths swaths_with_headlands_;

    sensor_msgs::msg::NavSatFix robot_loc_;
    sensor_msgs::msg::NavSatFix field_loc_;
    nav_msgs::msg::Path path_;

    farmbot_interfaces::msg::Segments segments_;
    visualization_msgs::msg::MarkerArray field_arrows_;
    visualization_msgs::msg::MarkerArray path_arrows_;
    geometry_msgs::msg::PolygonStamped outer_polygon_;
    geometry_msgs::msg::PolygonStamped inner_polygon_;

    rclcpp::TimerBase::SharedPtr visualization_timer_;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    rclcpp::Client<farmbot_interfaces::srv::Enu2Gps>::SharedPtr enu2gps_client_;
    rclcpp::Client<farmbot_interfaces::srv::GoToField>::SharedPtr goto_field_client_;
    rclcpp::Client<farmbot_interfaces::srv::GetTheField>::SharedPtr get_the_field_client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr field_arrows_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_arrows_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr outer_polygon_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr inner_polygon_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr loc_sub_;


public:
    FieldProcessorNode() : Node("ab_planner",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {
        name = this->get_parameter_or<std::string>("name", "ab_planner");
        topic_prefix_param = this->get_parameter_or<std::string>("topic_prefix", "/fb");
        vehicle_width_ = this->get_parameter_or<double>("vehicle_width", 0.5);
        vehicle_coverage_ = this->get_parameter_or<double>("vehicle_coverage", 3.0);
        alternate_freq_ = this->get_parameter_or<int>("alternate_freq", 1);
        path_angle_ = this->get_parameter_or<double>("path_angle", 90);
        goto_field_ = this->get_parameter_or<bool>("goto_field", true);

        // Create the service client
        gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>(topic_prefix_param + "/loc/gps2enu");
        enu2gps_client_ = this->create_client<farmbot_interfaces::srv::Enu2Gps>(topic_prefix_param + "/loc/enu2gps");
        goto_field_client_ = this->create_client<farmbot_interfaces::srv::GoToField>(topic_prefix_param + "/pln/get_route");
        get_the_field_client_ = this->create_client<farmbot_interfaces::srv::GetTheField>(topic_prefix_param + "/pln/get_field" );

        // Create the path publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_prefix_param + "/pln/path", 10);
        inner_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_prefix_param + "/pln/inner_field", 10);
        outer_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_prefix_param + "/pln/outer_field", 10);
        field_arrows_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_prefix_param + "/pln/arrow_swath", 10);
        path_arrows_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_prefix_param + "/pln/arrow_path", 10);

        // Create the location subscriber
        loc_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(topic_prefix_param + "/loc/fix", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            robot_loc_ = *msg;
            gps_locked_ = true;
        });

        // Timers
        visualization_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FieldProcessorNode::on_service_start_timer, this));

        //put runner in different thread
        std::thread([this] { runner(); }).detach();
    }

private:
    void on_service_start_timer() {
        if (!planner_initialized_) {
            return;
        }
        path_publisher_->publish(path_);
        outer_polygon_publisher_->publish(outer_polygon_);
        inner_polygon_publisher_->publish(inner_polygon_);
        field_arrows_pub_->publish(field_arrows_);
        if (goto_field_) {
            path_arrows_pub_->publish(path_arrows_);
        }
    }

    void runner() {
        auto getfield = get_the_field();
        if (getfield.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No points found");
            return;
        }
        auto field_in_enu = nav_to_enu(getfield);
        if (field_in_enu.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No ENU points received.");
            return;
        }
        process_field2(field_in_enu);

        // if (goto_field_) {
        //     set_initial_point();
        //     while (!gps_locked_) {
        //         RCLCPP_INFO(this->get_logger(), "Waiting for GPS lock...");
        //         std::this_thread::sleep_for(1s);
        //     }
        //     auto waypoints = go_to_field(robot_loc_, field_loc_);
        //     if (waypoints.empty()) {
        //         RCLCPP_ERROR(this->get_logger(), "No waypoints generated.");
        //         return;
        //     }
        //     auto waypoints_in_enu = nav_to_enu(waypoints);
        //     if (waypoints_in_enu.empty()) {
        //         RCLCPP_ERROR(this->get_logger(), "No ENU waypoints received.");
        //         return;
        //     }
        //     process_path(waypoints_in_enu);
        // }
        planner_initialized_ = true;
    }

    void process_field2(std::vector<std::pair<double, double>> points) {
        field_.gen_field(points);
        farmtrax::Field hl = field_.get_buffered(vehicle_width_* 2.0, farmtrax::BufferType::SHRINK);
        RCLCPP_INFO(this->get_logger(), "Field generated: %lu", field_.get_border_points().size());

        outer_polygon_ = vector2Polygon(field_.get_border_points());
        inner_polygon_ = vector2Polygon(hl.get_border_points());
        swaths_.gen_swaths(field_, hl, vehicle_coverage_, path_angle_, alternate_freq_, vehicle_width_);
        path_ = vector2Path(swaths_.get_swaths());
        // field_arrows_ = vector2Arrows(swaths_.get_swaths());
        RCLCPP_INFO(this->get_logger(), "Swaths generated: %lu", swaths_.get_swaths().size());

        mesh_ = farmtrax::Mesh(swaths_);
        mesh_.build_graph();
        RCLCPP_INFO(this->get_logger(), "Mesh generated");

        mesh_.to_dot("/home/bresilla/mesh.dot");
        RCLCPP_INFO(this->get_logger(), "Mesh saved to /home/bresilla/mesh.dot");

        pathh_ = farmtrax::Path(mesh_);
        std::vector<std::string> uuid_path = pathh_.find_optimal(farmtrax::Path::AlgorithmType::BRUTE_FORCE);
        std::vector<farmtrax::Swath> swath_path = pathh_.gen_swaths(uuid_path);
        field_arrows_ = vector2ArrowsColor(swath_path);
        pathh_.print_path(uuid_path);
    }

    void process_field(std::vector<std::pair<double, double>> points) {
        field_.gen_field(points);
        farmtrax::Field hl = field_.get_buffered(vehicle_width_* 2.0, farmtrax::BufferType::SHRINK);
        RCLCPP_INFO(this->get_logger(), "Field generated: %lu", field_.get_border_points().size());

        outer_polygon_ = vector2Polygon(field_.get_border_points());
        inner_polygon_ = vector2Polygon(hl.get_border_points());
        swaths_.gen_swaths(field_, hl, vehicle_coverage_, path_angle_, alternate_freq_, vehicle_width_);
        path_ = vector2Path(swaths_.get_swaths());
        // field_arrows_ = vector2Arrows(swaths_.get_swaths());
        RCLCPP_INFO(this->get_logger(), "Swaths generated: %lu", swaths_.get_swaths().size());

        planner_.gen_route(swaths_, vehicle_coverage_*alternate_freq_);
        swaths_with_headlands_ = planner_.get_swaths();
        // path_ = vector2Path(swaths_with_headlands.get_swaths());
        field_arrows_ = vector2ArrowsColor(swaths_with_headlands_.get_swaths());
        RCLCPP_INFO(this->get_logger(), "Swaths with headlands generated: %lu", swaths_with_headlands_.get_swaths().size());
    }

    void process_path(std::vector<std::pair<double, double>> points){
        path_arrows_ = vector2ArrowsColor(points);
    }

    void set_initial_point() {
        auto initial_points = swaths_with_headlands_.get_swaths()[0].swath.front();
        std::vector<std::pair<double, double>> path_points;
        std::pair<double, double> point = std::make_pair(initial_points.x(), initial_points.y());
        path_points.push_back(point);
        auto gps_points = enu_to_nav(path_points);
        if (gps_points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No GPS points received.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Initial point: %f, %f", gps_points[0][0], gps_points[0][1]);
        field_loc_.latitude = gps_points[0][0];
        field_loc_.longitude = gps_points[0][1];
    }

    std::vector<std::vector<double>> get_the_field(std::string geojson_file_path = "") {
        std::vector<std::vector<double>> points;
        auto request = std::make_shared<farmbot_interfaces::srv::GetTheField::Request>();
        request->geojson_file = geojson_file_path;
        while (!get_the_field_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return std::vector<std::vector<double>>();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = get_the_field_client_->async_send_request(request);
        auto navpts =  result.get()->field;
        for (const auto& point : navpts) {
            points.emplace_back(std::vector<double>{point.latitude, point.longitude});
        }
        return points;
    }

    std::vector<std::vector<double>> go_to_field(sensor_msgs::msg::NavSatFix start, sensor_msgs::msg::NavSatFix end) {
        std::vector<std::vector<double>> points;
        auto request = std::make_shared<farmbot_interfaces::srv::GoToField::Request>();
        request->start = start;
        request->end = end;
        while (!goto_field_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return std::vector<std::vector<double>>();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = goto_field_client_->async_send_request(request);
        auto navpts =  result.get()->waypoints;
        for (const auto& point : navpts) {
            points.emplace_back(std::vector<double>{point.latitude, point.longitude});
        }
        return points;
    }

    std::vector<std::pair<double, double>> nav_to_enu(std::vector<std::vector<double>> navpts){
        std::vector<std::pair<double, double>> points;
        auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();
        for (const auto& point : navpts) {
            sensor_msgs::msg::NavSatFix gps_point;
            gps_point.latitude = point[0];
            gps_point.longitude = point[1];
            gps_point.altitude = 0.0;  // Adjust if altitude data is available
            request->gps.push_back(gps_point);
        }
        while (!gps2enu_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return std::vector<std::pair<double, double>>();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = gps2enu_client_->async_send_request(request);
        auto getres =  result.get()->enu;
        for (const auto& point : getres) {
            points.emplace_back(std::pair<double, double>{point.position.x, point.position.y});
        }
        return points;
    }

    std::vector<std::vector<double>> enu_to_nav(std::vector<std::pair<double, double>> navpts){
        std::vector<std::vector<double>> points;
        auto request = std::make_shared<farmbot_interfaces::srv::Enu2Gps::Request>();
        for (const auto& point : navpts) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = point.first;
            pose.position.y = point.second;
            pose.position.z = 0.0;  // Adjust if altitude data is available
            request->enu.push_back(pose);
        }
        while (!enu2gps_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return std::vector<std::vector<double>>();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = enu2gps_client_->async_send_request(request);
        auto getres =  result.get()->gps;
        for (const auto& point : getres) {
            points.emplace_back(std::vector<double>{point.latitude, point.longitude});
        }
        return points;
    }

    // vector of vector of double to ros polygon
    geometry_msgs::msg::PolygonStamped vector2Polygon(const std::vector<std::pair<double, double>>& points) {
        geometry_msgs::msg::PolygonStamped polygon;
        polygon.header.frame_id = "map";
        polygon.header.stamp = rclcpp::Clock().now();
        for (const auto& point : points) {
            geometry_msgs::msg::Point32 p;
            p.x = point.first;
            p.y = point.second;
            polygon.polygon.points.push_back(p);
        }
        return polygon;
    }


    nav_msgs::msg::Path vector2Path(const std::vector<farmtrax::Swath>& swaths) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = rclcpp::Clock().now();
        for (const auto& swath : swaths) {
            for (const auto& point : swath.swath) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = point.x();
                pose.pose.position.y = point.y();
                path.poses.push_back(pose);
            }
        }
        return path;
    }

    visualization_msgs::msg::MarkerArray vector2Arrows(const std::vector<farmtrax::Swath>& swaths) {
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;
        for (const auto& swath : swaths) {
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = rclcpp::Clock().now();
            arrow.ns = "swath_arrows";
            arrow.id = id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.scale.x = 0.2;  // Shaft diameter
            arrow.scale.y = 1;  // Head diameter
            arrow.scale.z = 2.0;  // Head length
            arrow.color.r = 0.0f;  // Green
            if (swath.type == farmtrax::SwathType::LINE) {
                arrow.color.r = 0.0f;  // Green
                arrow.color.g = 1.0f;
                arrow.color.b = 1.0f;
                arrow.color.a = 1.0f;  // Fully opaque
            } else {
                arrow.color.r = 1.0f;  // Red
                arrow.color.g = 0.0f;
                arrow.color.b = 0.0f;
                arrow.color.a = 1.0f;  // Fully opaque
            }
            arrow.lifetime = rclcpp::Duration::from_seconds(0);
            geometry_msgs::msg::Point p1;
            p1.x = swath.swath[0].x();
            p1.y = swath.swath[0].y();
            geometry_msgs::msg::Point p2;
            p2.x = swath.swath[1].x();
            p2.y = swath.swath[1].y();
            if (swath.direction == farmtrax::Direction::REVERSE) {
                std::swap(p1, p2);
            }
            arrow.points.push_back(p1);
            arrow.points.push_back(p2);
            markers.markers.push_back(arrow);
        }
        return markers;
    }

    visualization_msgs::msg::MarkerArray vector2ArrowsColor(const std::vector<farmtrax::Swath>& swaths) {
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;
        int num_swaths = swaths.size();  // Total number of swaths
        for (const auto& swath : swaths) {
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = rclcpp::Clock().now();
            arrow.ns = "swath_arrows";
            arrow.id = id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.scale.x = 0.2;  // Shaft diameter
            arrow.scale.y = 1;    // Head diameter
            arrow.scale.z = 2.0;  // Head length
            // Calculate color based on the current id and total number of swaths
            float ratio = static_cast<float>(id) / num_swaths;  // Normalize id to [0, 1]
            if (ratio < 0.5) {
                // Orange (1.0, 0.5, 0.0) to Cyan (0.0, 1.0, 1.0) transition
                arrow.color.r = 1.0f - 2.0f * ratio;  // Decrease red
                arrow.color.g = 0.5f + 1.0f * ratio;  // Increase green
                arrow.color.b = 2.0f * ratio;         // Increase blue
            } else {
                // Cyan (0.0, 1.0, 1.0) to Magenta (1.0, 0.0, 1.0) transition
                arrow.color.r = 2.0f * (ratio - 0.5f);  // Increase red
                arrow.color.g = 1.0f - 2.0f * (ratio - 0.5f);  // Decrease green
                arrow.color.b = 1.0f;  // Blue stays at max
            }
            arrow.color.a = 1.0f;  // Fully opaque
            arrow.lifetime = rclcpp::Duration::from_seconds(0);
            geometry_msgs::msg::Point p1;
            p1.x = swath.swath[0].x();
            p1.y = swath.swath[0].y();
            geometry_msgs::msg::Point p2;
            p2.x = swath.swath[1].x();
            p2.y = swath.swath[1].y();
            if (swath.direction == farmtrax::Direction::REVERSE) {
                std::swap(p1, p2);
            }
            arrow.points.push_back(p1);
            arrow.points.push_back(p2);
            markers.markers.push_back(arrow);
        }
        return markers;
    }

    visualization_msgs::msg::MarkerArray vector2ArrowsColor(const std::vector<std::pair<double, double>> pairs) {
        RCLCPP_INFO(this->get_logger(), "Generating arrows for %lu pairs", pairs.size());
        visualization_msgs::msg::MarkerArray markers;
        int num_swaths = pairs.size()-1;  // Total number of swaths
        for (int id = 0; id < num_swaths; id++) {
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = rclcpp::Clock().now();
            arrow.ns = "path_arrows";
            arrow.id = id;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.scale.x = 0.2;  // Shaft diameter
            arrow.scale.y = 1;    // Head diameter
            arrow.scale.z = 2.0;  // Head length
            // Calculate color based on the current id and total number of swaths
            float ratio = static_cast<float>(id) / num_swaths;  // Normalize id to [0, 1]
            if (ratio > 0.5) {
                // Orange (1.0, 0.5, 0.0) to Cyan (0.0, 1.0, 1.0) transition
                arrow.color.r = 1.0f - 2.0f * ratio;  // Decrease red
                arrow.color.g = 0.5f + 1.0f * ratio;  // Increase green
                arrow.color.b = 2.0f * ratio;         // Increase blue
            } else {
                // Cyan (0.0, 1.0, 1.0) to Magenta (1.0, 0.0, 1.0) transition
                arrow.color.r = 2.0f * (ratio - 0.5f);  // Increase red
                arrow.color.g = 1.0f - 2.0f * (ratio - 0.5f);  // Decrease green
                arrow.color.b = 1.0f;  // Blue stays at max
            }
            arrow.color.a = 1.0f;  // Fully opaque
            arrow.lifetime = rclcpp::Duration::from_seconds(0);
            geometry_msgs::msg::Point p1;
            p1.x = pairs[id].first;
            p1.y = pairs[id].second;
            geometry_msgs::msg::Point p2;
            p2.x = pairs[id+1].first;
            p2.y = pairs[id+1].second;
            arrow.points.push_back(p1);
            arrow.points.push_back(p2);
            markers.markers.push_back(arrow);
        }
        return markers;
    }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<FieldProcessorNode>();
    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), e.what());
    }
    rclcpp::shutdown();
    return 0;
}
