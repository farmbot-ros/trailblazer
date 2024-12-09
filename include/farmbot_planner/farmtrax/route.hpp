#ifndef ROUTE_HPP
#define ROUTE_HPP

#include "farmbot_planner/farmtrax/swath.hpp"
#include "mesh.hpp"
#include <boost/graph/graph_traits.hpp>
#include <rclcpp/visibility_control.hpp>
#include <vector>
#include <string>
#include <unordered_set>
#include <stack>
#include <algorithm>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>
#include "spdlog/spdlog.h"
namespace echo = spdlog;

#include "rclcpp/rclcpp.hpp"

namespace farmtrax {
    class Route {
        private:
            rclcpp::Node::SharedPtr node_;          // ROS 2 node handle
            Mesh mesh_;
            Point start_point_;
            Point end_point_;
            bool start_set_;
            bool end_set_;
            std::vector<Swath> swaths; // Stores the resulting path as Swath objects

        public:
            // Enum to define different algorithm types
            enum class Algorithm {
                A_STAR,
                EXHAUSTIVE_SEARCH,
                BREADTH_FIRST_SEARCH,
                DEPTH_FIRST_SEARCH,
            };

            // Default constructor
            Route()  = default;

            void pass_node(rclcpp::Node::SharedPtr node) {
                node_ = node;
            }

            // Method to set the initial and final points
            void set_points(const Point& start, const Point& end) {
                start_point_ = start;
                end_point_ = end;
                start_set_ = true;
                end_set_ = true;
            }

            // Set only the start point
            void set_start_point(const Point& start) {
                start_point_ = start;
                start_set_ = true;
            }

            // Set only the end point
            void set_end_point(const Point& end) {
                end_point_ = end;
                end_set_ = true;
            }

            // Method to select default start and end points
            void select_default_points() {
                if (!start_set_) {
                    if (boost::num_vertices(mesh_.graph_) == 0) {
                        throw std::runtime_error("Mesh graph has no vertices to select as start point.");
                    }
                    // Assuming VertexList is boost::vecS, vertex descriptors are integers starting from 0
                    start_point_ = mesh_.graph_[0].point;
                    start_set_ = true;
                    std::cout << "Default start point selected: (" << start_point_.x() << ", " << start_point_.y() << ")\n";
                }

                if (!end_set_) {
                    if (boost::num_vertices(mesh_.graph_) == 0) {
                        throw std::runtime_error("Mesh graph has no vertices to select as end point.");
                    }
                    // Get the last vertex descriptor
                    auto last_vertex = boost::num_vertices(mesh_.graph_) - 1;
                    end_point_ = mesh_.graph_[last_vertex].point;
                    end_set_ = true;
                    std::cout << "Default end point selected: (" << end_point_.x() << ", " << end_point_.y() << ")\n";
                }
            }
            std::vector<Swath> get_swaths() const {
                return swaths;
            }

            std::string print_path() const {
                std::string path_str;
                for (const auto& swath : swaths) {
                    path_str += swath.uuid + " -> ";
                }
                if (!path_str.empty()) {
                    path_str.pop_back(); // Remove the last space
                    path_str.pop_back(); // Remove the last '>'
                    path_str.pop_back(); // Remove the last ' '
                    path_str.pop_back(); // Remove the last '-'
                    path_str.pop_back(); // Remove the last '>'
                }
                return path_str;
            }

            auto find_vertex_by_point(const Point& p) -> boost::graph_traits<Mesh::Graph>::vertex_descriptor {
                boost::graph_traits<Mesh::Graph>::vertex_iterator vi, vi_end;
                double min_distance = std::numeric_limits<double>::infinity();
                boost::graph_traits<Mesh::Graph>::vertex_descriptor closest_vertex = boost::graph_traits<Mesh::Graph>::null_vertex();
                for (boost::tie(vi, vi_end) = boost::vertices(mesh_.graph_); vi != vi_end; ++vi) {
                    double dist = bg::distance(mesh_.graph_[*vi].point, p);
                    if (dist < min_distance) {
                        min_distance = dist;
                        closest_vertex = *vi;
                    }
                }
                return closest_vertex;
            };

            void find_optimal(Mesh mesh, Algorithm type) {
                mesh_ = mesh;
                select_default_points(); // Set start and end points
                switch (type) {
                    case Algorithm::A_STAR: {
                        echo::info("A* algorithm not implemented yet.");
                        break;
                    }
                    case Algorithm::BREADTH_FIRST_SEARCH: {
                        echo::info("Breadth First Search algorithm not implemented yet.");
                        break;
                    }
                    case Algorithm::DEPTH_FIRST_SEARCH: {
                        echo::info("Depth First Search algorithm not implemented yet.");
                        break;
                    }
                    case Algorithm::EXHAUSTIVE_SEARCH: {
                        echo::info("Starting exhaustive search...");
                        exhaustive_search();
                        break;
                    }
                    default:
                        throw std::invalid_argument("Unsupported algorithm type.");
                        swaths = std::vector<Swath>();
                        break;
                }
            }


            // Brute Force Implementation: Exhaustive Search
            std::vector<Swath> exhaustive_search() {
                // Vector to store the path as a sequence of Swath objects
                std::vector<Swath> path;
                // Set to keep track of visited swaths to ensure each swath is traversed only once
                std::unordered_set<std::string> visited_swaths;
                // Find the starting vertex in the mesh graph using the start_point_
                boost::graph_traits<Mesh::Graph>::vertex_descriptor start_vertex = find_vertex_by_point(start_point_);
                // Find the ending vertex in the mesh graph using the end_point_
                boost::graph_traits<Mesh::Graph>::vertex_descriptor end_vertex = find_vertex_by_point(end_point_);

                // Count the total number of swaths (edges of type LINE) that need to be visited
                size_t total_swaths = 0;
                boost::graph_traits<Mesh::Graph>::edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::edges(mesh_.graph_); ei != ei_end; ++ei) {
                    const EdgeProperties& props = mesh_.graph_[*ei];
                    if (props.swath.type == SwathType::LINE) {
                        total_swaths++;
                    }
                }
                echo::info("Total swaths to visit: {}", total_swaths);

                //print the number of vertices in the graph
                echo::info("Number of vertices in the graph: {}", boost::num_vertices(mesh_.graph_));

                //print the number of edges in the graph
                echo::info("Number of edges in the graph: {}", boost::num_edges(mesh_.graph_));

                // Initialize an invalid vertex descriptor to track the previous vertex (no previous vertex initially)
                boost::graph_traits<Mesh::Graph>::vertex_descriptor invalid_vertex = boost::graph_traits<Mesh::Graph>::null_vertex();

                // Start the recursive exhaustive search from the start_vertex
                int depth = 0;
                auto found = recursive_func(
                    start_vertex,       // Current vertex
                    end_vertex,         // Destination vertex
                    visited_swaths,    // Set of visited swaths
                    path,               // Current path
                    total_swaths,      // Total number of swaths to visit
                    invalid_vertex,     // Previous vertex (none at the start)
                    depth               // Depth of the search tree
                );

                if (found) {
                    echo::info("Path found with {} swaths.", path.size());
                    swaths = path; // Store the path
                    return path; // Return the path containing Swath objects
                } else {
                    echo::error("Path not found.");
                    return std::vector<Swath>(); // Return empty path
                }
            }

        private:

        bool recursive_func(
                boost::graph_traits<Mesh::Graph>::vertex_descriptor current_vertex,    // Current position in the graph
                boost::graph_traits<Mesh::Graph>::vertex_descriptor end_vertex,        // Destination vertex
                std::unordered_set<std::string>& visited_swaths,                       // Set of already visited swaths
                std::vector<Swath>& path,                                              // Current path as a sequence of Swath objects
                size_t total_swaths,                                                   // Total number of swaths to visit
                boost::graph_traits<Mesh::Graph>::vertex_descriptor previous_vertex,   // Previously visited vertex to prevent immediate backtracking
                int depth                                                             // Depth of the recursion
            ) {
                // Base Case: If all swaths have been visited return true
                if (visited_swaths.size() == total_swaths) { return true; }

                depth++;
                // echo::info("Depth: {}", depth);

                boost::graph_traits<Mesh::Graph>::out_edge_iterator ei2, ei_end2;
                std::vector<boost::graph_traits<Mesh::Graph>::edge_descriptor> line_edges;
                std::vector<boost::graph_traits<Mesh::Graph>::edge_descriptor> turn_edges;
                for (boost::tie(ei2, ei_end2) = boost::out_edges(current_vertex, mesh_.graph_); ei2 != ei_end2; ++ei2) {
                    auto edge = *ei2;
                    const EdgeProperties& props = mesh_.graph_[edge];
                    if (props.swath.type == SwathType::LINE) {
                        line_edges.push_back(edge);
                    } else {
                        turn_edges.push_back(edge);
                    }
                }

                for (const auto& edge : line_edges) {
                    const EdgeProperties& props = mesh_.graph_[edge];
                    boost::graph_traits<Mesh::Graph>::vertex_descriptor next_vertex = boost::target(edge, mesh_.graph_);
                    const std::string& swath_uuid = props.swath.uuid;
                    if (
                        (next_vertex == previous_vertex) ||
                        (visited_swaths.find(swath_uuid) != visited_swaths.end())
                    ) { continue; }

                    path.push_back(props.swath);
                    visited_swaths.insert(swath_uuid);
                    if (recursive_func(next_vertex, end_vertex, visited_swaths, path, total_swaths, current_vertex, depth)) {
                        return true;
                    }
                    path.pop_back();
                    visited_swaths.erase(swath_uuid);
                }

                for (const auto& pair : turn_edges) {
                    const EdgeProperties& props = mesh_.graph_[pair];
                    boost::graph_traits<Mesh::Graph>::vertex_descriptor next_vertex = boost::target(pair, mesh_.graph_);
                    const std::string& swath_uuid = props.swath.uuid;
                    if (
                        (next_vertex == previous_vertex)
                    ) { continue; }
                    path.push_back(props.swath);
                    if (recursive_func(next_vertex, end_vertex, visited_swaths, path, total_swaths, current_vertex, depth)) {
                        return true;
                    }
                    path.pop_back();
                }

                //backtrack
                depth--;
                // echo::info("Depth: {}", depth);

                return false; // No valid path found from this vertex
            }

    };
} // namespace farmtrax

#endif // ROUTE_HPP
