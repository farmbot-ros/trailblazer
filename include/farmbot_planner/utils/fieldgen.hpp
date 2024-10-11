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
#include "farmbot_interfaces/srv/gps2_enu.hpp"

#include "farmbot_interfaces/msg/waypoint.hpp"
#include "farmbot_interfaces/msg/segment.hpp"
#include "farmbot_interfaces/msg/segments.hpp"



using namespace std::chrono_literals;

namespace field {

    struct PathSegment {
        std::pair<double, double> start;
        std::pair<double, double> end;
        std::vector<std::pair<double, double>> middle;
    };

    class Field {
        private:
            farmbot_interfaces::msg::Segments segments_;

            F2CCells field_;
            F2CCells head_;
            F2CRobot vehicle_;
            F2CRoute route_;
            F2CPath path_;
            double finness_;
        
        public:
            Field() {}
            ~Field() {}

            Field(double width, double cov_width, double finness) {
                vehicle_.setWidth(width);
                vehicle_.setCovWidth(cov_width);
                finness_ = finness;
            }

            void vehicle_params(double width, double cov_width, double finness) {
                vehicle_.setWidth(width);
                vehicle_.setCovWidth(cov_width);
                finness_ = finness;
            }

            F2CPath gen_path(const std::vector<geometry_msgs::msg::Pose>& enu_points) {
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
                F2CSwathsByCells swaths = bf.generateSwaths(M_PI, vehicle_.getCovWidth(), head_);
                // Generate the route
                f2c::rp::RoutePlannerBase route_planner;
                route_ = route_planner.genRoute(head_, swaths);
                // Generate the path
                f2c::pp::DubinsCurves dubins_;
                f2c::pp::PathPlanning path_planner_;
                path_ = path_planner_.planPath(vehicle_, route_, dubins_);
                path_.reduce(finness_);
                return path_;
            }

            std::vector<PathSegment> gen_pathsegments(const F2CPath& path) {
                std::vector<PathSegment> pathSegments;
                PathSegment segm;
                auto last_state_type = f2c::types::PathSectionType::HL_SWATH;
                f2c::types::PathState prevState; // Define the previous state properly
                for (const auto& state : path) {
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

            farmbot_interfaces::msg::Segments gen_segments(const F2CPath& path) {
                return gen_segments(gen_pathsegments(path));
            }

            void visualizePath(std::string filename) {
                f2c::Visualizer::figure();
                f2c::Visualizer::plot(field_);
                f2c::Visualizer::plot(head_);
                f2c::Visualizer::plot(path_);
                f2c::Visualizer::figure_size(2400, 2400);
                f2c::Visualizer::save(filename);
            }

    };

} // namespace field