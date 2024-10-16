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

    class Route {
    private:
        Swaths swaths_;

        // Define the Boost Graph types
        typedef boost::adjacency_list<
            boost::listS,                  // OutEdgeList
            boost::vecS,                   // VertexList
            boost::directedS,              // Directed graph
            boost::property<boost::vertex_index_t, int>, // Vertex properties
            boost::property<boost::edge_weight_t, double> // Edge properties
        > Graph;

        typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef boost::graph_traits<Graph>::edge_descriptor Edge;

        Graph graph_;
        std::map<Point, Vertex> point_vertex_map_; // Map from Point to Vertex

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
        Route(const Swaths& swaths) : swaths_(swaths) {
            build_graph();
            compute_shortest_traversal();
        }

        // Function to build the graph from the swaths
        void build_graph() {
            const std::vector<Swath>& swaths = swaths_.get_swaths();

            // Create vertices for all unique points
            for (const auto& swath : swaths) {
                const Point& start_point = swath.swath.front();
                const Point& end_point = swath.swath.back();

                // Create or get the vertex for the start point
                Vertex start_vertex = get_or_create_vertex(start_point);
                // Create or get the vertex for the end point
                Vertex end_vertex = get_or_create_vertex(end_point);

                // Add the swath edge (from start to end) with appropriate cost
                double swath_cost = 0.0; // Moving along a swath is cheap
                Edge swath_edge;
                bool inserted;
                boost::tie(swath_edge, inserted) = boost::add_edge(start_vertex, end_vertex, graph_);
                graph_[swath_edge] = swath_cost;
            }

            // Add transition edges between swaths
            for (const auto& swath_from : swaths) {
                const Point& from_point = swath_from.swath.back(); // End point of current swath
                Vertex from_vertex = get_or_create_vertex(from_point);

                for (const auto& swath_to : swaths) {
                    // Skip if same swath
                    if (swath_from.uuid == swath_to.uuid) continue;

                    const Point& to_point = swath_to.swath.front(); // Start point of next swath
                    Vertex to_vertex = get_or_create_vertex(to_point);

                    // Add a transition edge from 'from_vertex' to 'to_vertex'
                    double transition_cost = bg::distance(from_point, to_point); // Costly to traverse
                    Edge transition_edge;
                    bool inserted;
                    boost::tie(transition_edge, inserted) = boost::add_edge(from_vertex, to_vertex, graph_);
                    graph_[transition_edge] = transition_cost;
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

        // Function to compute the traversal path covering all swaths
        void compute_shortest_traversal() {
            // Use a greedy nearest neighbor heuristic for simplicity
            const std::vector<Swath>& swaths = swaths_.get_swaths();
            if (swaths.empty()) return;

            std::set<std::string> visited_swaths;
            const Swath* current_swath = &swaths[0];
            visited_swaths.insert(current_swath->uuid);

            // Add the first swath to traversal
            traversal_.push_back({ current_swath->swath, current_swath->type, current_swath->uuid });

            while (visited_swaths.size() < swaths.size()) {
                const Point& current_point = current_swath->swath.back(); // End point of current swath
                const Swath* next_swath = nullptr;
                double min_cost = std::numeric_limits<double>::max();

                // Find the nearest unvisited swath
                for (const auto& swath : swaths) {
                    if (visited_swaths.count(swath.uuid) > 0) continue;

                    double cost = bg::distance(current_point, swath.swath.front());
                    if (cost < min_cost) {
                        min_cost = cost;
                        next_swath = &swath;
                    }
                }

                if (next_swath) {
                    // Add transition path (TURN)
                    LineString transition_path;
                    transition_path.push_back(current_point);
                    transition_path.push_back(next_swath->swath.front());

                    // Create a Swath for the transition
                    Swath transition_swath;
                    transition_swath.swath = transition_path;
                    transition_swath.uuid = generate_UUID();
                    transition_swath.type = SwathType::TURN;
                    transition_swath.transportlane = true; // Assuming transitions are transport lanes
                    transition_swath.length = bg::length(transition_path);
                    transition_swath.direction = Direction::FORWARD; // Direction may not be critical here

                    // Add the transition swath to traversal
                    traversal_.push_back({ transition_swath.swath, transition_swath.type, transition_swath.uuid });

                    // Add the next swath to traversal
                    traversal_.push_back({ next_swath->swath, next_swath->type, next_swath->uuid });
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
    };

} // namespace farmtrax

#endif // ROUTE_HPP
