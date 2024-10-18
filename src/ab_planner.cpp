#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include <iostream>
#include "farmbot_planner/utils/geojson.hpp"
#include "farmbot_planner/farmtrax/field.hpp"
#include "farmbot_planner/farmtrax/swath.hpp"
#include "farmbot_planner/farmtrax/route.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
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
    farmtrax::Route route_;

    sensor_msgs::msg::NavSatFix loc_;
    nav_msgs::msg::Path path_;
    farmbot_interfaces::msg::Segments segments_;
    visualization_msgs::msg::Marker line_marker_;
    visualization_msgs::msg::MarkerArray arrow_marker_;
    geometry_msgs::msg::PolygonStamped outer_polygon_;
    geometry_msgs::msg::PolygonStamped inner_polygon_;

    rclcpp::TimerBase::SharedPtr visualization_timer_;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    rclcpp::Client<farmbot_interfaces::srv::Enu2Gps>::SharedPtr enu2gps_client_;
    rclcpp::Client<farmbot_interfaces::srv::GoToField>::SharedPtr goto_field_client_;
    rclcpp::Client<farmbot_interfaces::srv::GetTheField>::SharedPtr get_the_field_client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrow_publisher_;
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
        goto_field_client_ = this->create_client<farmbot_interfaces::srv::GoToField>(topic_prefix_param + "/pla/get_route");
        get_the_field_client_ = this->create_client<farmbot_interfaces::srv::GetTheField>(topic_prefix_param + "/pla/get_field" );

        // Create the path publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_prefix_param + "/pla/path", 10);
        inner_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_prefix_param + "/pla/inner_field", 10);
        outer_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_prefix_param + "/pla/outer_field", 10);
        arrow_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_prefix_param + "/pla/swath_arrows", 10);
        line_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_prefix_param + "/pla/swath_lines", 10);

        // Create the location subscriber
        loc_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(topic_prefix_param + "/loc/fix", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            loc_ = *msg;
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
        line_publisher_->publish(line_marker_);
        arrow_publisher_->publish(arrow_marker_);
    }

    void runner() {
        auto getfield = get_the_field();
        if (getfield.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No points found");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Perimeter of the field is %lu points", getfield.size());
        auto enu_points = nav_to_pose(getfield);
        if (enu_points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No ENU points received.");
            return;
        }
        process_enu_points(enu_points);
        planner_initialized_ = true;

        if (goto_field_) {
            while (!gps_locked_) {
                RCLCPP_INFO(this->get_logger(), "Waiting for GPS lock...");
                std::this_thread::sleep_for(1s);
            }
            auto start = loc_;
            auto end = sensor_msgs::msg::NavSatFix();
            end.latitude = getfield[0][0];
            end.longitude = getfield[0][1];
            auto waypoints = go_to_field(start, end);
            if (waypoints.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No waypoints generated.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Going to the field are %lu waypoints", waypoints.size());
        }
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

    std::vector<geometry_msgs::msg::Pose> nav_to_pose(std::vector<std::vector<double>> navpts) {
        std::vector<geometry_msgs::msg::Pose> poses;
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
                return std::vector<geometry_msgs::msg::Pose>();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = gps2enu_client_->async_send_request(request);
        auto enupts =  result.get()->enu;
        for (const auto& point : enupts) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = point.position.x;
            pose.position.y = point.position.y;
            poses.push_back(pose);
        }
        return poses;
    }

    std::vector<std::pair<double, double>> nav_to_double(std::vector<std::vector<double>> navpts){
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
        auto enupts =  result.get()->enu;
        for (const auto& point : enupts) {
            points.emplace_back(std::pair<double, double>{point.position.x, point.position.y});
        }
        return points;
    }
       

    void process_enu_points(const std::vector<geometry_msgs::msg::Pose>& enu_points) {
        if (enu_points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No ENU points received.");
            return;
        }

        std::vector<std::pair<double, double>> points;
        for (const auto& point : enu_points) {
            points.emplace_back(point.position.x, point.position.y);
        }

        field_.gen_field(points);
        farmtrax::Field hl = field_.get_buffered(vehicle_width_* 2.0, farmtrax::BufferType::SHRINK);
        RCLCPP_INFO(this->get_logger(), "Field generated: %lu", field_.get_border_points().size());

        outer_polygon_ = vector2Polygon(field_.get_border_points());
        inner_polygon_ = vector2Polygon(hl.get_border_points());
        swaths_.gen_swaths(field_, hl, vehicle_coverage_, path_angle_, alternate_freq_, vehicle_width_);
        path_ = vector2Path(swaths_.get_swaths());
        // arrow_marker_ = vector2Arrows(swaths_.get_swaths());
        RCLCPP_INFO(this->get_logger(), "Swaths generated: %lu", swaths_.get_swaths().size());


        route_.gen_route(swaths_);
        auto swaths_with_headlands = route_.get_swaths();
        // outer_polygon_ = vector2Polygon(route_.get_headland_points());
        // path_ = vector2Path(swaths_with_headlands.get_swaths());
        arrow_marker_ = vector2ArrowsColor(swaths_with_headlands.get_swaths());
        RCLCPP_INFO(this->get_logger(), "Swaths with headlands generated: %lu", swaths_with_headlands.get_swaths().size());

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

    visualization_msgs::msg::Marker vector2Lines(const std::vector<farmtrax::Swath>& swaths) {
        visualization_msgs::msg::Marker line_marker;
        // Set the frame and timestamp
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = rclcpp::Clock().now();
        // Set the namespace and id for this marker
        line_marker.ns = "swath_lines";
        line_marker.id = 0;
        // Define the type of marker (LINE_STRIP)
        line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        // Set the scale of the lines (width of the lines)
        line_marker.scale.x = 0.1;  // Adjust the width of the line here
        // Set the lifetime of the marker (0 means forever)
        line_marker.lifetime = rclcpp::Duration::from_seconds(0);
        // Iterate through the swaths and add points to the marker
        for (const auto& swath : swaths) {
            geometry_msgs::msg::Point p1;
            p1.x = swath.swath[0].x();
            p1.y = swath.swath[0].y();
            geometry_msgs::msg::Point p2;
            p2.x = swath.swath[1].x();
            p2.y = swath.swath[1].y();
            if (swath.type == farmtrax::SwathType::LINE) {
                line_marker.color.r = 0.0f;  // Green
                line_marker.color.g = 1.0f;
                line_marker.color.b = 1.0f;
                line_marker.color.a = 1.0f;  // Fully opaque
            } else {
                line_marker.color.r = 1.0f;  // Red
                line_marker.color.g = 0.0f;
                line_marker.color.b = 0.0f;
                line_marker.color.a = 1.0f;  // Fully opaque
            }
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
        }
        return line_marker;
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