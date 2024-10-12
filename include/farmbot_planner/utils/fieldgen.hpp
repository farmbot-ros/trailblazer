#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include <iostream>
#include "fields2cover.h"
#include "farmbot_planner/utils/geojson.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"

#include "farmbot_interfaces/msg/waypoint.hpp"
#include "farmbot_interfaces/msg/segment.hpp"
#include "farmbot_interfaces/msg/segments.hpp"



using namespace std::chrono_literals;

namespace field {

        // Function to convert degrees to radians
    double toRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    // Function to convert radians to degrees
    double toDegrees(double radians) {
        return radians * 180.0 / M_PI;
    }

    struct PathSegment {
        std::pair<double, double> start;
        std::pair<double, double> end;
        std::vector<std::pair<double, double>> middle;
    };

    class Field {
        private:
            std::vector<geometry_msgs::msg::Pose> poses_;
            F2CCells field_;
            F2CCells head_;
            F2CRobot vehicle_;
            // F2CSwathsByCells swaths_;
            F2CSwaths swaths_;
            F2CRoute route_;
            F2CPath path_;
            double finness_; 
            int angle_;
        
        public:
            Field() {}
            ~Field() {}

            Field(double width, double cov_width, double finness, double angle = 90) {
                vehicle_.setWidth(width);
                vehicle_.setCovWidth(cov_width);
                finness_ = finness;
                if (angle < 0 || angle > 180) {
                    angle_ = 90;
                } else {
                    angle_ = angle;
                }
            }

            void vehicle_params(double width, double cov_width, double finness) {
                vehicle_.setWidth(width);
                vehicle_.setCovWidth(cov_width);
                finness_ = finness;
            }

            F2CPath gen_path(const std::vector<geometry_msgs::msg::Pose>& enu_points) {
                poses_ = enu_points;
                F2CLinearRing ring_ = F2CLinearRing();
                for (const auto& enu_point : enu_points) {
                    ring_.addPoint(F2CPoint(enu_point.position.x, enu_point.position.y));
                }
                // Create the field object
                field_ = F2CCells(F2CCell{ring_});
                // Generate the headlands
                f2c::hg::ConstHL const_hl;
                head_ = const_hl.generateHeadlands(field_, 3.0 * vehicle_.getWidth());
                // Generate waaths
                f2c::sg::BruteForce bf;
                swaths_ = bf.generateSwaths(toRadians(angle_), vehicle_.getCovWidth(), head_.getGeometry(0));
                // swaths_ = bf.generateSwaths(toRadians(angle_), vehicle_.getCovWidth(), head_);
                // Sort the swaths
                // f2c::rp::SnakeOrder snake_sorter;
                // swaths_ = snake_sorter.genSortedSwaths(swaths_);
                // Generate the route
                // f2c::rp::RoutePlannerBase route_planner;
                // route_ = route_planner.genRoute(head_, swaths_);
                // Generate the path
                f2c::pp::DubinsCurves dubins_;
                f2c::pp::PathPlanning path_planner_;
                path_ = path_planner_.planPath(vehicle_, swaths_, dubins_);
                path_.reduce(finness_);
                return path_;
            }

            void update_route(double x, double y, double z=0) {
                f2c::types::Point p(x, y, z);
                F2CSwaths swaths2;
                const double tolerance = 0.1;
                for (auto& swath : swaths_) {
                    while (std::abs(swath.startPoint().X() - p.X()) > tolerance || 
                        std::abs(swath.startPoint().Y() - p.Y()) > tolerance) {
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current point: %f, %f", swath.startPoint().X(), swath.startPoint().Y());
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Target point: %f, %f", p.X(), p.Y());
                        continue;
                    }
                    swaths2.emplace_back(swath);
                }
                swaths_ = swaths2;
                f2c::pp::DubinsCurves dubins_;
                f2c::pp::PathPlanning path_planner_;
                path_ = path_planner_.planPath(vehicle_, swaths_, dubins_);
                path_.reduce(finness_);
                std::cout << "Route updated" << std::endl;
            }

            std::vector<PathSegment> gen_pathsegments() {
                std::vector<PathSegment> pathSegments;
                PathSegment segm;
                auto last_state_type = f2c::types::PathSectionType::HL_SWATH;
                f2c::types::PathState prevState; // Define the previous state properly
                for (const auto& state : path_) {
                    if (state.type == f2c::types::PathSectionType::SWATH) {
                        // If the last state was a TURN, complete the segment and store it
                        if (last_state_type == f2c::types::PathSectionType::TURN) {
                            segm.start = segm.middle.front();
                            segm.middle.erase(segm.middle.begin());
                            segm.end = {state.point.X(), state.point.Y()};
                            pathSegments.push_back(segm);
                            // Clear the segment for the next one
                            segm = PathSegment();
                            segm.middle.clear();
                        }
                        // Store the SWATH start point
                    } else if (state.type == f2c::types::PathSectionType::TURN) {
                        // If the last state was a SWATH, start a new segment
                        if (last_state_type == f2c::types::PathSectionType::SWATH) {
                            segm.start = {prevState.point.X(), prevState.point.Y()};
                            segm.end = {state.point.X(), state.point.Y()};
                            pathSegments.push_back(segm);
                            // Clear the segment for the next one
                            segm = PathSegment();
                            segm.middle.clear();
                        }
                        // Add the TURN points to the middle vector
                        segm.middle.push_back({state.point.X(), state.point.Y()});
                    }
                    // Update the previous state and last state type
                    prevState = state;
                    last_state_type = state.type;
                }
                // Make sure to handle the last segment if it exists
                if (!segm.middle.empty() || last_state_type == f2c::types::PathSectionType::SWATH) {
                    pathSegments.push_back(segm);
                }
                return pathSegments;
            }

            farmbot_interfaces::msg::Segments gen_segments(const std::vector<field::PathSegment>& pathSegments) {
                auto segments_ = farmbot_interfaces::msg::Segments();
                for (const auto& segment : pathSegments) {
                    farmbot_interfaces::msg::Segment seg;
                    seg.origin.pose.position.x = segment.start.first;
                    seg.origin.pose.position.y = segment.start.second;
                    seg.destination.pose.position.x = segment.end.first;
                    seg.destination.pose.position.y = segment.end.second;
                    for (const auto& middle_point : segment.middle) {
                        farmbot_interfaces::msg::Waypoint middle;
                        middle.pose.position.x = middle_point.first;
                        middle.pose.position.y = middle_point.second;
                        seg.inbetween.push_back(middle);
                    }
                    segments_.segments.push_back(seg);
                }
                return segments_;
            }

            nav_msgs::msg::Path gen_path_msg() {
                nav_msgs::msg::Path path_msg;
                path_msg.header.frame_id = "map";
                path_msg.header.stamp = rclcpp::Clock().now();
                for (const auto& state : path_) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path_msg.header;
                    pose.pose.position.x = state.point.X();
                    pose.pose.position.y = state.point.Y();
                    path_msg.poses.push_back(pose);
                }
                return path_msg;
            }

            farmbot_interfaces::msg::Segments gen_segments() {
                return gen_segments(gen_pathsegments());
            }

            std::pair<geometry_msgs::msg::PolygonStamped, geometry_msgs::msg::PolygonStamped> get_polygons() {
                geometry_msgs::msg::PolygonStamped outer;
                geometry_msgs::msg::PolygonStamped inner;
                outer.header.frame_id = "map";
                outer.header.stamp = rclcpp::Clock().now();
                inner.header = outer.header;
                for (const auto& cell : field_) {
                    for (const auto& ring : cell) {
                        for (const auto& point : ring) {
                            geometry_msgs::msg::Point32 p;
                            p.x = point.X();
                            p.y = point.Y();
                            outer.polygon.points.push_back(p);
                        }
                    }
                }
                for (const auto& cell : head_) {
                    for (const auto& ring : cell) {
                        for (const auto& point : ring) {
                            geometry_msgs::msg::Point32 p;
                            p.x = point.X();
                            p.y = point.Y();
                            inner.polygon.points.push_back(p);
                        }
                    }
                }
                return std::make_pair(outer, inner);
            }

    };

} // namespace field