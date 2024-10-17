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
            set_swaths(swaths);
        }

        // Set the swaths for the route
        void set_swaths(const Swaths& swaths) {
            swaths_ = swaths;
            headland_path_ = swaths.get_connecting_polygon();
            build_graph();
            compute_shortest_traversal();
        }

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

        // Function to project point 'p' onto segment 'seg' and store the result in 'projected_p'
        void project_point_onto_segment(const Point& p, const bg::model::segment<Point>& seg, Point& projected_p) {
            double x1 = seg.first.x();
            double y1 = seg.first.y();
            double x2 = seg.second.x();
            double y2 = seg.second.y();
            double x0 = p.x();
            double y0 = p.y();

            double dx = x2 - x1;
            double dy = y2 - y1;
            double length_squared = dx * dx + dy * dy;

            // If the segment is a point
            if (length_squared == 0.0)
            {
                projected_p = seg.first;
                return;
            }

            // Parameter t indicates the position of the projection on the segment
            double t = ((x0 - x1) * dx + (y0 - y1) * dy) / length_squared;

            // Clamp t to the segment range [0, 1]
            if (t < 0.0)
                t = 0.0;
            else if (t > 1.0)
                t = 1.0;

            // Compute the projected point
            projected_p.x(x1 + t * dx);
            projected_p.y(y1 + t * dy);
        }

        // Function to compute the distance along the perimeter between two points
        double perimeter_distance(const Point& p1, const Point& p2){
            // Get the outer ring of the polygon
            const auto& ring = bg::exterior_ring(headland_path_);

            // Compute cumulative distances along the ring
            std::vector<double> cumulative_lengths(ring.size(), 0.0);
            for (size_t i = 1; i < ring.size(); ++i)
            {
                double segment_length = bg::distance(ring[i - 1], ring[i]);
                cumulative_lengths[i] = cumulative_lengths[i - 1] + segment_length;
            }

            double total_perimeter_length = cumulative_lengths.back();

            // Helper function to compute the position along the perimeter
            auto compute_position_along_perimeter = [&](const Point& p) -> double
            {
                // Check if the point lies exactly on any vertex
                for (size_t i = 0; i < ring.size(); ++i)
                {
                    if (bg::equals(p, ring[i]))
                    {
                        return cumulative_lengths[i];
                    }
                }

                double min_distance = std::numeric_limits<double>::max();
                double position = -1.0;

                // Iterate over segments to find the closest projection
                for (size_t i = 1; i < ring.size(); ++i)
                {
                    bg::model::segment<Point> seg(ring[i - 1], ring[i]);
                    Point projected_p;
                    project_point_onto_segment(p, seg, projected_p);
                    double dist = bg::distance(p, projected_p);

                    // Check if the projected point lies on the segment
                    if (bg::covered_by(projected_p, seg))
                    {
                        if (dist < min_distance)
                        {
                            min_distance = dist;
                            double dist_to_p = cumulative_lengths[i - 1] + bg::distance(ring[i - 1], projected_p);
                            position = dist_to_p;
                        }
                    }
                }

                // If position is still -1, the point is not projected onto any segment
                if (position == -1.0)
                {
                    throw std::runtime_error("Point cannot be projected onto the polygon perimeter.");
                }

                return position;
            };

            double pos_p1 = compute_position_along_perimeter(p1);
            double pos_p2 = compute_position_along_perimeter(p2);

            // Compute distances along the perimeter in both directions
            double dist_clockwise = std::abs(pos_p2 - pos_p1);
            double dist_counter_clockwise = total_perimeter_length - dist_clockwise;

            // Return the minimal distance
            return std::min(dist_clockwise, dist_counter_clockwise);
        }

        // Function to compute the traversal path covering all swaths
        void compute_shortest_traversal() {
            // Use a greedy nearest neighbor heuristic for simplicity
            const std::vector<Swath>& swaths = swaths_.get_swaths();
            if (swaths.empty()) return;

            std::set<std::string> visited_swaths;
            const Swath* current_swath = &swaths[0];
            visited_swaths.insert(current_swath->uuid);

            // Add the first swath to traversal and new_swaths_
            traversal_.push_back({ current_swath->swath, current_swath->type, current_swath->uuid });
            new_swaths_.add_swath(*current_swath); // Add to new_swaths_

            while (visited_swaths.size() < swaths.size()) {
                Point current_point;
                if (current_swath->direction == Direction::FORWARD) {
                    current_point = current_swath->swath.back(); // End point
                } else if (current_swath->direction == Direction::REVERSE) {
                    current_point = current_swath->swath.front(); // Start point (since direction is reversed)
                } else {
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
                        continue; // Handle unexpected direction value
                    }

                    double cost = bg::distance(current_point, swath_start_point);
                    if (cost < min_cost) {
                        min_cost = cost;
                        next_swath = &swath;
                    }
                }

                if (next_swath) {
                    // Add transition path (TURN)
                    LineString transition_path;
                    transition_path.push_back(current_point);

                    Point next_swath_start_point;
                    if (next_swath->direction == Direction::FORWARD) {
                        next_swath_start_point = next_swath->swath.front(); // Start point
                    } else if (next_swath->direction == Direction::REVERSE) {
                        next_swath_start_point = next_swath->swath.back(); // End point
                    } else {
                        break; // Handle unexpected direction value
                    }
                    transition_path.push_back(next_swath_start_point);

                    // Create a Swath for the transition
                    Swath transition_swath;
                    transition_swath.swath = transition_path;
                    transition_swath.uuid = generate_UUID();
                    transition_swath.type = SwathType::TURN;
                    transition_swath.transportlane = true; // Assuming transitions are transport lanes
                    transition_swath.length = bg::length(transition_path);
                    transition_swath.direction = Direction::FORWARD; // Direction may not be critical here

                    // Add the transition swath to traversal and new_swaths_
                    traversal_.push_back({ transition_swath.swath, transition_swath.type, transition_swath.uuid });
                    new_swaths_.add_swath(transition_swath); // Add to new_swaths_

                    // Add the next swath to traversal and new_swaths_
                    traversal_.push_back({ next_swath->swath, next_swath->type, next_swath->uuid });
                    new_swaths_.add_swath(*next_swath); // Add to new_swaths_

                    visited_swaths.insert(next_swath->uuid);
                    current_swath = next_swath;
                } else {
                    // No unvisited swaths found
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

        LineString create_segment(const Point& start_point, const Point& end_point) {
            LineString segment;
            segment.push_back(start_point);
            segment.push_back(end_point);
            return segment;
        }

        //get the new swaths
        Swaths get_swaths() const {
            return new_swaths_;
        }

    };

} // namespace farmtrax

#endif // ROUTE_HPP
