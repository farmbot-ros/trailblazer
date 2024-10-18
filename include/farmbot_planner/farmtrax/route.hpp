#ifndef ROUTE_HPP
#define ROUTE_HPP

#include "field.hpp"
#include "swath.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/algorithms/distance.hpp>

// Include Boost Graph Library
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <vector>
#include <string>
#include <map>
#include <set>
#include <limits>
#include <iostream>

namespace farmtrax {
    namespace bg = boost::geometry;

    // Define Cartesian point type
    typedef bg::model::d2::point_xy<double> Point;
    // Define linestring type for edges
    typedef bg::model::linestring<Point> LineString;
    // Polygon type for field boundary
    typedef bg::model::polygon<Point> Polygon;

    // Comparator for Point
    struct PointComparator {
        bool operator()(const Point& lhs, const Point& rhs) const {
            // Use a tolerance for floating-point comparisons
            const double epsilon = 1e-9;

            if (std::abs(lhs.x() - rhs.x()) > epsilon) {
                return lhs.x() < rhs.x();
            } else {
                return lhs.y() < rhs.y();
            }
        }
    };

    class Route {
        private:
            Swaths swaths_;                    // Original swaths provided to the Route
            Swaths new_swaths_;                // Swaths including transitions (new swaths)
            Polygon headland_path_;            // Field boundary polygon
            double swath_width_;               // Total distance of the route

            // Define the Boost Graph types
            typedef boost::adjacency_list<
                boost::listS,                  // OutEdgeList
                boost::vecS,                   // VertexList
                boost::directedS,              // Directed graph
                boost::no_property,            // Vertex properties
                boost::property<boost::edge_weight_t, double> // Edge properties
            > Graph;

            typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
            typedef boost::graph_traits<Graph>::edge_descriptor Edge;

            Graph graph_;
            std::map<Point, Vertex, PointComparator> point_vertex_map_; // Map from Point to Vertex

            // Structure to store the traversal path
            struct TraversalStep {
                LineString path;               // The path to traverse
                SwathType type;                // Type of the swath (LINE, TURN, etc.)
                std::string swath_uuid;        // UUID of the swath (if applicable)
            };

            std::vector<TraversalStep> traversal_;

        public:
            Route() = default;

            // Constructor that initializes the route with swaths
            Route(const Swaths& swaths){
                gen_route(swaths);
            }

            // Set the swaths for the route
            void gen_route(const Swaths& swaths, double swath_width = 1.0) {
                swaths_ = swaths;
                swath_width_ = swath_width;
                headland_path_ = swaths.get_connecting_polygon();
                build_graph();
                compute_shortest_traversal();
            }

            //get the new swaths
            Swaths get_swaths() const {
                return new_swaths_;
            }

        private:
            // Function to build the graph from the swaths
            void build_graph() {
                const std::vector<Swath>& swaths = swaths_.get_swaths();

                // Create the edge weight property map
                typedef boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap;
                EdgeWeightMap edge_weight_map = get(boost::edge_weight, graph_);

                // Create vertices for all unique points
                for (const auto& swath : swaths) {
                    const Point& start_point = swath.swath.front();
                    const Point& end_point = swath.swath.back();

                    // Create or get the vertex for the start point
                    Vertex start_vertex = get_or_create_vertex(start_point);
                    // Create or get the vertex for the end point
                    Vertex end_vertex = get_or_create_vertex(end_point);

                    // Add the swath edge with appropriate direction based on swath.direction
                    double swath_cost = 0.0; // Moving along a swath is cheap
                    Edge swath_edge;
                    bool inserted;

                    if (swath.direction == Direction::FORWARD) {
                        // Add edge from start to end
                        boost::tie(swath_edge, inserted) = boost::add_edge(start_vertex, end_vertex, graph_);
                    } else if (swath.direction == Direction::REVERSE) {
                        // Add edge from end to start
                        boost::tie(swath_edge, inserted) = boost::add_edge(end_vertex, start_vertex, graph_);
                    } else {
                        continue; // Handle unexpected direction value if necessary
                    }

                    // Set the edge weight using the property map
                    edge_weight_map[swath_edge] = swath_cost;
                }

                // Add transition edges between swaths
                for (const auto& swath_from : swaths) {
                    // Determine the appropriate point based on the direction
                    Vertex from_vertex;
                    Point from_point;
                    if (swath_from.direction == Direction::FORWARD) {
                        from_point = swath_from.swath.back(); // End point
                    } else if (swath_from.direction == Direction::REVERSE) {
                        from_point = swath_from.swath.front(); // Start point (since direction is reversed)
                    } else {
                        continue; // Handle unexpected direction value if necessary
                    }
                    from_vertex = get_or_create_vertex(from_point);

                    for (const auto& swath_to : swaths) {
                        // Skip if same swath
                        if (swath_from.uuid == swath_to.uuid) continue;

                        Vertex to_vertex;
                        Point to_point;
                        if (swath_to.direction == Direction::FORWARD) {
                            to_point = swath_to.swath.front(); // Start point
                        } else if (swath_to.direction == Direction::REVERSE) {
                            to_point = swath_to.swath.back(); // End point (since direction is reversed)
                        } else {
                            continue; // Handle unexpected direction value if necessary
                        }
                        to_vertex = get_or_create_vertex(to_point);

                        // Add a transition edge from 'from_vertex' to 'to_vertex'
                        double transition_cost = perimeter_distance(from_point, to_point);
                        Edge transition_edge;
                        bool inserted;
                        boost::tie(transition_edge, inserted) = boost::add_edge(from_vertex, to_vertex, graph_);

                        // Set the edge weight using the property map
                        edge_weight_map[transition_edge] = transition_cost;
                    }
                }
            }

            // Function to get or create a vertex for a point
            Vertex get_or_create_vertex(const Point& point) {
                auto it = point_vertex_map_.find(point);
                if (it != point_vertex_map_.end()) {
                    return it->second;
                } else {
                    Vertex v = boost::add_vertex(graph_);
                    point_vertex_map_[point] = v;
                    return v;
                }
            }

            double perimeter_distance(const Point& p1, const Point& p2){
                const auto& ring = bg::exterior_ring(headland_path_);
                // Function to find the position of a point along the perimeter
                auto find_position = [&](const Point& p) -> double {
                    double cumulative_length = 0.0;
                    for (size_t i = 0; i < ring.size() - 1; ++i){
                        const Point& seg_start = ring[i];
                        const Point& seg_end = ring[i + 1];
                        // Check if the point lies on the current segment
                        if (bg::covered_by(p, bg::model::segment<Point>(seg_start, seg_end))) {
                            // Calculate the distance from seg_start to p
                            double distance_to_p = bg::distance(seg_start, p);
                            return cumulative_length + distance_to_p;
                        }
                        // Add the length of the current segment to cumulative_length
                        cumulative_length += bg::distance(seg_start, seg_end);
                    }
                    throw std::invalid_argument("Point is not on the perimeter of the polygon.");
                };

                // Find positions of p1 and p2 along the perimeter
                double pos_p1 = find_position(p1);
                double pos_p2 = find_position(p2);
                // Calculate total perimeter length
                double total_perimeter = 0.0;
                for (size_t i = 0; i < ring.size() - 1; ++i){
                    total_perimeter += bg::distance(ring[i], ring[i + 1]);
                }
                // Calculate clockwise and counter-clockwise distances
                double dist_clockwise = std::abs(pos_p2 - pos_p1);
                double dist_counter_clockwise = total_perimeter - dist_clockwise;
                // Return the minimal distance
                return std::min(dist_clockwise, dist_counter_clockwise);
            }


            LineString remove_colinear(const LineString& path) {
                if (path.size() < 3)
                    return path; // No colinear points possible

                LineString simplified;
                simplified.push_back(path[0]); // Always include the first point

                for (size_t i = 1; i < path.size() - 1; ++i) {
                    const Point& prev = path[i - 1];
                    const Point& current = path[i];
                    const Point& next = path[i + 1];

                    // Calculate vectors
                    double dx1 = bg::get<0>(current) - bg::get<0>(prev);
                    double dy1 = bg::get<1>(current) - bg::get<1>(prev);
                    double dx2 = bg::get<0>(next) - bg::get<0>(current);
                    double dy2 = bg::get<1>(next) - bg::get<1>(current);

                    // Compute cross product to determine colinearity
                    double cross = dx1 * dy2 - dy1 * dx2;

                    if (std::abs(cross) > 1e-10) { // Threshold to account for floating-point errors
                        simplified.push_back(current); // Not colinear, keep the point
                    }
                    // Else: Colinear, skip the current point
                }

                simplified.push_back(path.back()); // Always include the last point
                return simplified;
            }

            LineString compute_perimeter_path(const Polygon& polygon, const Point& start, const Point& end) {
                const auto& exterior = polygon.outer();
                LineString path_cw, path_ccw;

                // Find indices of start and end points on the perimeter
                int start_idx = -1, end_idx = -1;
                for (size_t i = 0; i < exterior.size(); ++i) {
                    if (bg::equals(exterior[i], start)) {
                        start_idx = static_cast<int>(i);
                    }
                    if (bg::equals(exterior[i], end)) {
                        end_idx = static_cast<int>(i);
                    }
                    if (start_idx != -1 && end_idx != -1) break;
                }

                // If exact points not found, return a direct transition as a fallback
                if (start_idx == -1 || end_idx == -1) {
                    LineString direct;
                    direct.emplace_back(start);
                    direct.emplace_back(end);
                    return direct;
                }

                // Construct Clockwise Path
                size_t i = static_cast<size_t>(start_idx);
                while (i != static_cast<size_t>(end_idx)) {
                    path_cw.emplace_back(exterior[i]);
                    i = (i + 1) % exterior.size();
                }
                path_cw.emplace_back(exterior[end_idx]);

                // Construct Counter-Clockwise Path
                i = static_cast<size_t>(start_idx);
                while (i != static_cast<size_t>(end_idx)) {
                    if (i == 0)
                        i = exterior.size() - 1;
                    else
                        i = (i - 1) % exterior.size();
                    path_ccw.emplace_back(exterior[i]);
                }
                path_ccw.emplace_back(exterior[end_idx]);

                // Calculate lengths of both paths
                double length_cw = bg::length(path_cw);
                double length_ccw = bg::length(path_ccw);

                // Select the shorter path
                LineString selected_path = (length_cw <= length_ccw) ? path_cw : path_ccw;

                // Remove colinear points to simplify the path
                LineString simplified_path = remove_colinear(selected_path);

                return simplified_path;
            }

            // Function to compute the traversal path covering all swaths with swath_width_ constraint
            void compute_shortest_traversal() {
                // Retrieve all swaths
                const std::vector<Swath>& swaths = swaths_.get_swaths();
                if (swaths.empty()) return;

                // Initialize visited swaths set
                std::unordered_set<std::string> visited_swaths;
                const Swath* current_swath = &swaths[0];
                visited_swaths.insert(current_swath->uuid);

                // Add the first swath to traversal and new_swaths_
                traversal_.push_back({ current_swath->swath, current_swath->type, current_swath->uuid });
                new_swaths_.add_swath(*current_swath); // Add to new_swaths_

                while (visited_swaths.size() < swaths.size()) {
                    Point current_point;
                    // Determine the current endpoint based on the swath direction
                    if (current_swath->direction == Direction::FORWARD) {
                        current_point = current_swath->swath.back(); // End point
                    } else if (current_swath->direction == Direction::REVERSE) {
                        current_point = current_swath->swath.front(); // Start point (since direction is reversed)
                    } else {
                        std::cerr << "Unexpected direction value.\n";
                        break; // Handle unexpected direction value
                    }

                    const Swath* next_swath = nullptr;
                    double min_cost = std::numeric_limits<double>::max();

                    // Find the nearest unvisited swath
                    for (const auto& swath : swaths) {
                        if (visited_swaths.count(swath.uuid) > 0) continue;

                        Point swath_start_point;
                        if (swath.direction == Direction::FORWARD) {
                            swath_start_point = swath.swath.front(); // Start point
                        } else if (swath.direction == Direction::REVERSE) {
                            swath_start_point = swath.swath.back(); // End point (since direction is reversed)
                        } else {
                            std::cerr << "Unexpected direction value in swaths.\n";
                            continue; // Handle unexpected direction value
                        }

                        double cost = bg::distance(current_point, swath_start_point);
                        if (cost < min_cost) {
                            min_cost = cost;
                            next_swath = &swath;
                        }
                    }

                    if (next_swath) {
                        // Compute direct transition path
                        LineString direct_transition;
                        direct_transition.push_back(current_point);
                        Point next_swath_start_point;
                        if (next_swath->direction == Direction::FORWARD) {
                            next_swath_start_point = next_swath->swath.front(); // Start point
                        } else if (next_swath->direction == Direction::REVERSE) {
                            next_swath_start_point = next_swath->swath.back(); // End point
                        } else {
                            std::cerr << "Unexpected direction value in next_swath.\n";
                            break; // Handle unexpected direction value
                        }
                        direct_transition.push_back(next_swath_start_point);

                        double transition_length = bg::length(direct_transition);

                        if (transition_length > (2.0 * swath_width_)) {
                            // Transition path is longer than 2 * swath_width_, use perimeter path

                            // Compute perimeter transition path (assumes compute_perimeter_path is defined)
                            LineString perimeter_transition = compute_perimeter_path(headland_path_, current_point, next_swath_start_point);

                            // Split the perimeter_transition into multiple two-point LineStrings
                            for (size_t i = 1; i < perimeter_transition.size(); ++i) {
                                LineString segment;
                                segment.push_back(perimeter_transition[i - 1]);
                                segment.push_back(perimeter_transition[i]);

                                // Create a Swath for each perimeter segment
                                Swath perimeter_swath;
                                perimeter_swath.swath = segment;
                                perimeter_swath.uuid = generate_UUID();
                                perimeter_swath.type = SwathType::TURN;
                                perimeter_swath.transportlane = true; // Assuming transitions are transport lanes
                                perimeter_swath.length = bg::length(segment);
                                perimeter_swath.direction = Direction::FORWARD; // Direction may not be critical here

                                // Add the perimeter transition swath segment to traversal and new_swaths_
                                traversal_.push_back({ perimeter_swath.swath, perimeter_swath.type, perimeter_swath.uuid });
                                new_swaths_.add_swath(perimeter_swath); // Add to new_swaths_
                            }

                        } else {
                            // Transition path is within the allowed swath width, use direct transition

                            // Create a Swath for the direct transition
                            Swath transition_swath;
                            transition_swath.swath = direct_transition;
                            transition_swath.uuid = generate_UUID();
                            transition_swath.type = SwathType::TURN;
                            transition_swath.transportlane = true; // Assuming transitions are transport lanes
                            transition_swath.length = transition_length;
                            transition_swath.direction = Direction::FORWARD; // Direction may not be critical here

                            // Add the direct transition swath to traversal and new_swaths_
                            traversal_.push_back({ transition_swath.swath, transition_swath.type, transition_swath.uuid });
                            new_swaths_.add_swath(transition_swath); // Add to new_swaths_
                        }

                        // Add the next swath to traversal and new_swaths_
                        traversal_.push_back({ next_swath->swath, next_swath->type, next_swath->uuid });
                        new_swaths_.add_swath(*next_swath); // Add to new_swaths_

                        // Mark the next swath as visited and update current_swath
                        visited_swaths.insert(next_swath->uuid);
                        current_swath = next_swath;
                    } else {
                        // No unvisited swaths found
                        std::cerr << "No unvisited swaths found. Ending traversal.\n";
                        break;
                    }
                }
            }


            // Function to generate a unique identifier
            std::string generate_UUID() const {
                return boost::uuids::to_string(boost::uuids::random_generator()());
            }

            // Function to get the traversal steps
            std::vector<TraversalStep> get_traversal() const {
                return traversal_;
            }

            // Function to output the traversal (e.g., for visualization)
            void output_traversal() const {
                for (const auto& step : traversal_) {
                    if (step.type == SwathType::LINE) {
                        std::cout << "Traverse Swath UUID: " << step.swath_uuid << std::endl;
                    } else if (step.type == SwathType::TURN) {
                        std::cout << "Transition Movement (TURN), UUID: " << step.swath_uuid << std::endl;
                    }
                    // Optionally, output the path coordinates
                    // std::cout << "Path coordinates: " << bg::wkt(step.path) << std::endl;
                }
            }

    };

} // namespace farmtrax

#endif // ROUTE_HPP
