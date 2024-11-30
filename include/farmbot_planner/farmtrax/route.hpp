// route.hpp

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
                        execute_exhaustive_search();
                        break;
                    }
                    default:
                        throw std::invalid_argument("Unsupported algorithm type.");
                        swaths = std::vector<Swath>();
                        break;
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

            void execute_exhaustive_search() {
                // Function to find the vertex descriptor corresponding to a point
                auto find_vertex_by_point = [&](const Point& p) -> boost::graph_traits<Mesh::Graph>::vertex_descriptor {
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

                    // Define a tolerance for considering two points as the same
                    const double tolerance = 1e-6;
                    if (min_distance < tolerance) {
                        return closest_vertex;
                    } else {
                        return boost::graph_traits<Mesh::Graph>::null_vertex();
                    }
                };

                // Find start and end vertices
                auto start_vertex = find_vertex_by_point(start_point_);
                auto end_vertex = find_vertex_by_point(end_point_);

                // Variables to store the minimal path
                double min_total_weight = std::numeric_limits<double>::infinity();
                std::vector<boost::graph_traits<Mesh::Graph>::edge_descriptor> min_path_edges;

                // Recursive function to find all paths
                std::function<void(boost::graph_traits<Mesh::Graph>::vertex_descriptor,
                                std::vector<boost::graph_traits<Mesh::Graph>::edge_descriptor>&,
                                double,
                                std::unordered_set<boost::graph_traits<Mesh::Graph>::vertex_descriptor>&)> find_paths;

                find_paths = [&](boost::graph_traits<Mesh::Graph>::vertex_descriptor current,
                                std::vector<boost::graph_traits<Mesh::Graph>::edge_descriptor>& path,
                                double total_weight,
                                std::unordered_set<boost::graph_traits<Mesh::Graph>::vertex_descriptor>& visited) {
                    // Add current vertex to visited
                    visited.insert(current);

                    // Check if we've reached the end vertex
                    if (current == end_vertex) {
                        if (total_weight < min_total_weight) {
                            min_total_weight = total_weight;
                            min_path_edges = path;
                        }
                        // Remove current vertex from visited before returning
                        visited.erase(current);
                        return;
                    }

                    // Iterate over all outgoing edges from the current vertex
                    boost::graph_traits<Mesh::Graph>::out_edge_iterator ei, ei_end;
                    for (boost::tie(ei, ei_end) = boost::out_edges(current, mesh_.graph_); ei != ei_end; ++ei) {
                        auto e = *ei;
                        auto target_v = boost::target(e, mesh_.graph_);

                        // Avoid cycles by checking if the target vertex has been visited
                        if (visited.find(target_v) == visited.end()) {
                            // Add edge to path
                            path.push_back(e);
                            // Accumulate edge weight
                            double edge_weight = mesh_.graph_[e].weight;
                            double new_total_weight = total_weight + edge_weight;

                            // Recursive call to continue path exploration
                            find_paths(target_v, path, new_total_weight, visited);

                            // Backtrack: remove edge from path
                            path.pop_back();
                        }
                    }

                    // Remove current vertex from visited before backtracking
                    visited.erase(current);
                };

                // Start the search
                std::vector<boost::graph_traits<Mesh::Graph>::edge_descriptor> path;
                std::unordered_set<boost::graph_traits<Mesh::Graph>::vertex_descriptor> visited;
                find_paths(start_vertex, path, 0.0, visited);

                if (min_path_edges.empty()) {
                    echo::error("No path found from start to end.");
                    swaths.clear();
                } else {
                    // Build the swaths vector from min_path_edges
                    swaths.clear();
                    for (const auto& e : min_path_edges) {
                        swaths.push_back(mesh_.graph_[e].swath);
                    }
                    echo::info("Optimal path found with total weight: {}", min_total_weight);
                }
            }
    };
} // namespace farmtrax

#endif // ROUTE_HPP
