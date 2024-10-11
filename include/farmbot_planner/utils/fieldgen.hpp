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
            farmbot_interfaces::msg::Segment segment_;
            F2CLinearRing ring_;
            F2CCells cells_;
            F2CCells no_hl_;
            F2CRobot vehicle_;
            F2CRoute route_;
            F2CPath path_;
            f2c::pp::PathPlanning path_planner_;
            f2c::pp::DubinsCurves dubins_;
            double finness_;
        
        public:

            Field() {}
            ~Field() {}


            Field(double width, double cov_width, double finness) {
                vehicle_.setWidth(width);
                vehicle_.setCovWidth(cov_width);
                finness_ = finness;
            }

            F2CPath generatePath(const std::vector<geometry_msgs::msg::Pose>& enu_points) {
                ring_ = F2CLinearRing();
                for (const auto& enu_point : enu_points) {
                    ring_.addPoint(F2CPoint(enu_point.position.x, enu_point.position.y));
                }

                cells_ = F2CCells(F2CCell{ring_});

                f2c::hg::ConstHL const_hl;
                no_hl_ = const_hl.generateHeadlands(cells_, 3.0 * vehicle_.getWidth());

                f2c::sg::BruteForce bf;
                F2CSwathsByCells swaths = bf.generateSwaths(M_PI, vehicle_.getCovWidth(), no_hl_);

                f2c::rp::RoutePlannerBase route_planner;
                route_ = route_planner.genRoute(no_hl_, swaths);

                f2c::pp::DubinsCurves dubins;
                path_ = path_planner_.planPath(vehicle_, route_, dubins);
                path_.reduce(finness_);

                return path_;
            }

            std::vector<PathSegment> pathSegmentGen(const F2CPath& path) {
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

            void visualizePath(std::string filename) {
                f2c::Visualizer::figure();
                f2c::Visualizer::plot(cells_);
                f2c::Visualizer::plot(no_hl_);
                f2c::Visualizer::plot(path_);
                f2c::Visualizer::figure_size(2400, 2400);
                f2c::Visualizer::save(filename);
            }

    };

} // namespace field