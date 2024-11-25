#ifndef PATH_HPP
#define PATH_HPP

#include "farmbot_planner/farmtrax/swath.hpp"
#include "mesh.hpp"
#include <boost/graph/graph_traits.hpp>
#include <vector>
#include <string>
#include <unordered_set>
#include <stack>
#include <algorithm>
#include <stdexcept>
#include <limits>
#include <cmath>
#include <iostream> // For debug output

namespace farmtrax {
    class Route {
        private:
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

                // Alternatively, implement another strategy, such as selecting vertices with min/max coordinates
                // Uncomment the following lines to use the min/max coordinate strategy

                /*
                // Strategy 2: Select vertices with minimum and maximum x + y coordinates
                auto min_max = std::minmax_element(boost::vertices(mesh_.graph_).first, boost::vertices(mesh_.graph_).second,
                    [&](const Mesh::Graph::vertex_descriptor a, const Mesh::Graph::vertex_descriptor b) -> bool {
                        const Point& pa = mesh_.graph_[a].point;
                        const Point& pb = mesh_.graph_[b].point;
                        return (pa.x() + pa.y()) < (pb.x() + pb.y());
                    });
                start_point_ = mesh_.graph_[*min_max.first].point;
                end_point_ = mesh_.graph_[*min_max.second].point;
                start_set_ = true;
                end_set_ = true;
                */
            }

            void find_optimal(Mesh mesh, Algorithm type) {
                mesh_ = mesh;
                select_default_points(); // Set start and end points
                switch (type) {
                    case Algorithm::A_STAR: {
                        throw std::invalid_argument("A* algorithm not implemented yet.");
                    }
                    case Algorithm::BREADTH_FIRST_SEARCH: {
                        throw std::invalid_argument("Breadth First Search algorithm not implemented yet.");
                    }
                    case Algorithm::DEPTH_FIRST_SEARCH: { // Corrected typo
                        throw std::invalid_argument("Depth First Search algorithm not implemented yet.");
                    }
                    case Algorithm::EXHAUSTIVE_SEARCH: {
                        swaths = exhaustive_search();
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
                }
                return path_str;
            }

        private:
            // Helper method to check if a vertex has any unvisited line edges
            bool has_unvisited_line_edge(boost::graph_traits<Mesh::Graph>::vertex_descriptor vertex, const std::unordered_set<std::string>& visited_swaths) const {
                boost::graph_traits<Mesh::Graph>::out_edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::out_edges(vertex, mesh_.graph_); ei != ei_end; ++ei) {
                    const EdgeProperties& props = mesh_.graph_[*ei];
                    if (props.swath.type == SwathType::LINE && visited_swaths.find(props.swath.uuid) == visited_swaths.end()) {
                        return true;
                    }
                }
                return false;
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
                std::cout << "Total swaths to visit: " << total_swaths << "\n";


                //print the number of vertices in the graph
                std::cout << "Number of vertices in the graph: " << boost::num_vertices(mesh_.graph_) << "\n";

                //print the number of edges in the graph
                std::cout << "Number of edges in the graph: " << boost::num_edges(mesh_.graph_) << "\n";

                //for each vertice, print the number of edges that are connected to it (in and out) and type of swath
                // for (boost::graph_traits<Mesh::Graph>::vertex_iterator vi = boost::vertices(mesh_.graph_).first; vi != boost::vertices(mesh_.graph_).second; ++vi) {
                //     std::cout << "Vertex " << *vi << " has " << boost::in_degree(*vi, mesh_.graph_) << " incoming edges and " << boost::out_degree(*vi, mesh_.graph_) << " outgoing edges.\n";
                //     for (boost::graph_traits<Mesh::Graph>::out_edge_iterator ei = boost::out_edges(*vi, mesh_.graph_).first; ei != boost::out_edges(*vi, mesh_.graph_).second; ++ei) {
                //         const EdgeProperties& props = mesh_.graph_[*ei];
                //         std::cout << "Edge " << *ei << " connected to vertex " << *vi << " is of type " << props.swath.type << "\n";
                //     }
                // }

                // Initialize an invalid vertex descriptor to track the previous vertex (no previous vertex initially)
                boost::graph_traits<Mesh::Graph>::vertex_descriptor invalid_vertex = boost::graph_traits<Mesh::Graph>::null_vertex();

                // Start the recursive exhaustive search from the start_vertex
                auto found = recursive_ex(
                    start_vertex,       // Current vertex
                    end_vertex,         // Destination vertex
                    visited_swaths,    // Set of visited swaths
                    path,               // Current path
                    total_swaths,      // Total number of swaths to visit
                    invalid_vertex     // Previous vertex (none at the start)
                );

                if (found) {
                    std::cout << "Valid path found.\n";
                    return path; // Directly return the path containing Swath objects
                } else {
                    throw std::runtime_error("No valid path found that visits all swaths.");
                }
            }

            bool recursive_ex2(
                    boost::graph_traits<Mesh::Graph>::vertex_descriptor current_vertex,
                    boost::graph_traits<Mesh::Graph>::vertex_descriptor end_vertex,
                    std::unordered_set<std::string>& visited_swaths,
                    std::vector<Swath>& path,
                    size_t total_swaths,
                    boost::graph_traits<Mesh::Graph>::vertex_descriptor previous_vertex
                ) {
                    // Base Case: If current vertex is the end vertex and all LINE swaths have been visited
                    if (visited_swaths.size() == total_swaths && current_vertex == end_vertex) {
                        return true; // Valid path found
                    }

                    // Retrieve all outgoing edges from the current vertex
                    boost::graph_traits<Mesh::Graph>::out_edge_iterator ei, ei_end;
                    std::vector<boost::graph_traits<Mesh::Graph>::edge_descriptor> line_edges;
                    std::vector<std::pair<boost::graph_traits<Mesh::Graph>::edge_descriptor, bool>> turn_edges_with_flag;

                    for (boost::tie(ei, ei_end) = boost::out_edges(current_vertex, mesh_.graph_); ei != ei_end; ++ei) {
                        auto edge = *ei;
                        const EdgeProperties& props = mesh_.graph_[edge];

                        if (props.swath.type == SwathType::LINE) {
                            line_edges.push_back(edge);
                        } else {
                            boost::graph_traits<Mesh::Graph>::vertex_descriptor next_vertex = boost::target(edge, mesh_.graph_);
                            bool has_unvisited_line = has_unvisited_line_edge(next_vertex, visited_swaths);
                            turn_edges_with_flag.emplace_back(edge, has_unvisited_line);
                        }
                    }

                    // Process LINE edges first
                    //
                    for (const auto& edge : line_edges) {
                        const EdgeProperties& props = mesh_.graph_[edge];
                        const std::string& swath_uuid = props.swath.uuid;
                        boost::graph_traits<Mesh::Graph>::vertex_descriptor next_vertex = boost::target(edge, mesh_.graph_);

                        // Prevent immediate backtracking
                        if (next_vertex == previous_vertex) {
                            continue;
                        }

                        // If the LINE swath has already been visited, skip
                        if (visited_swaths.find(swath_uuid) != visited_swaths.end()) {
                            continue;
                        }

                        // Visit the LINE swath
                        visited_swaths.insert(swath_uuid);
                        path.push_back(props.swath);

                        // Recursive call
                        if (recursive_ex2(next_vertex, end_vertex, visited_swaths, path, total_swaths, current_vertex)) {
                            return true;
                        }

                        // Backtrack
                        visited_swaths.erase(swath_uuid);
                        path.pop_back();
                    }

                    // Process TURN edges that lead to vertices with unvisited LINE edges
                    for (const auto& pair : turn_edges_with_flag) {
                        if (!pair.second) {
                            continue;
                        }
                        auto edge = pair.first;
                        const EdgeProperties& props = mesh_.graph_[edge];
                        boost::graph_traits<Mesh::Graph>::vertex_descriptor next_vertex = boost::target(edge, mesh_.graph_);

                        // Prevent immediate backtracking
                        if (next_vertex == previous_vertex) {
                            continue;
                        }

                        // Add the TURN swath to the path
                        path.push_back(props.swath);

                        // Recursive call
                        if (recursive_ex2(next_vertex, end_vertex, visited_swaths, path, total_swaths, current_vertex)) {
                            return true;
                        }

                        // Backtrack
                        path.pop_back();
                    }

                    // Process TURN edges that do not lead to unvisited LINE edges
                    for (const auto& pair : turn_edges_with_flag) {
                        if (pair.second) {
                            continue;
                        }
                        auto edge = pair.first;
                        const EdgeProperties& props = mesh_.graph_[edge];
                        boost::graph_traits<Mesh::Graph>::vertex_descriptor next_vertex = boost::target(edge, mesh_.graph_);

                        // Prevent immediate backtracking
                        if (next_vertex == previous_vertex) {
                            continue;
                        }

                        // Add the TURN swath to the path
                        path.push_back(props.swath);

                        // Recursive call
                        if (recursive_ex2(next_vertex, end_vertex, visited_swaths, path, total_swaths, current_vertex)) {
                            return true;
                        }

                        // Backtrack
                        path.pop_back();
                    }

                    // No valid path found from this vertex
                    return false;
                }
            // Recursive function to find a path through the graph
            bool recursive_ex(
                boost::graph_traits<Mesh::Graph>::vertex_descriptor current_vertex,    // Current position in the graph
                boost::graph_traits<Mesh::Graph>::vertex_descriptor end_vertex,        // Destination vertex
                std::unordered_set<std::string>& visited_swaths,                       // Set of already visited swaths
                std::vector<Swath>& path,                                              // Current path as a sequence of Swath objects
                size_t total_swaths,                                                   // Total number of swaths to visit
                boost::graph_traits<Mesh::Graph>::vertex_descriptor previous_vertex    // Previously visited vertex to prevent immediate backtracking
            ) {
                // Base Case: If current vertex is the end vertex and all swaths have been visited
                if (current_vertex == end_vertex && visited_swaths.size() == total_swaths) {
                    return true; // Path successfully found
                }

                std::vector<Swath> new_path;

                // Iterate over all outgoing edges from the current_vertex
                boost::graph_traits<Mesh::Graph>::out_edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::out_edges(current_vertex, mesh_.graph_); ei != ei_end; ++ei) {
                    auto edge = *ei;
                    const EdgeProperties& props = mesh_.graph_[edge]; // Retrieve properties of the edge
                    std::string swath_uuid = props.swath.uuid;  // Get the UUID of the swath associated with this edge

                    // Determine if the edge is a swath (of type LINE)
                    Direction edge_direction = props.swath.direction;
                    SwathType edge_type = props.swath.type;
                    bool is_swath = (edge_type == SwathType::LINE);
                    bool can_forward = (edge_direction == Direction::FORWARD);

                    // If the edge is a swath and has already been visited or is not in the correct direction, skip it
                    if (is_swath) {
                        // if (!can_forward) {continue; }
                        if (visited_swaths.find(swath_uuid) != visited_swaths.end()) {
                            continue; // Skip this edge as it's already been traversed
                        }
                    }

                    // Identify the target vertex of the edge
                    auto target_vertex = boost::target(edge, mesh_.graph_);

                    // Prevent immediate backtracking to the previous vertex via a non-swath edge (e.g., a turn)
                    if (!is_swath && target_vertex == previous_vertex) {
                        continue; // Skip to avoid redundant traversal
                    }

                    // Choose to traverse this edge
                    if (is_swath) {
                        // Mark the swath as visited and add it to the current path
                        visited_swaths.insert(swath_uuid);
                        new_path.push_back(props.swath); // Add Swath object directly
                    } else {
                        // For non-swath edges (e.g., turns), simply log the traversal
                        // new_path.push_back(props.swath); // Add Swath object directly
                    }

                    // Recurse with the updated current vertex and previous vertex
                    bool found = recursive_ex(
                        target_vertex,      // Move to the target vertex
                        end_vertex,         // Destination remains the same
                        visited_swaths,    // Updated set of visited swaths
                        new_path,               // Updated path
                        total_swaths,      // Total swaths to visit
                        current_vertex      // Current vertex becomes the previous vertex for the next recursion
                    );

                    if (found) {
                        path.insert(path.end(), new_path.begin(), new_path.end());
                        return true; // Propagate the success up the recursion stack
                    }

                    // Backtrack: If the path wasn't successful, undo the traversal
                    if (is_swath) {
                        visited_swaths.erase(swath_uuid); // Unmark the swath as visited
                    }
                    path.pop_back();                   // Remove the swath from the current path
                }

                return false; // No valid path found from this vertex
            }

            // Function to find a vertex by its associated point
            boost::graph_traits<Mesh::Graph>::vertex_descriptor find_vertex_by_point(const Point& point) const {
                boost::graph_traits<Mesh::Graph>::vertex_iterator vi, vi_end;
                for (boost::tie(vi, vi_end) = boost::vertices(mesh_.graph_); vi != vi_end; ++vi) {
                    if (bg::equals(mesh_.graph_[*vi].point, point)) {
                        return *vi;
                    }
                }
                throw std::runtime_error("Point not found in mesh graph.");
            }
    };
} // namespace farmtrax

#endif // PATH_HPP
