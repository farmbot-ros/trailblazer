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
            Route() : mesh_(), start_set_(false), end_set_(false) {}

            // Constructor that initializes with a Mesh instance
            Route(Mesh mesh) : mesh_(mesh), start_set_(false), end_set_(false) {
                // No need to populate uuid_to_swath_map_ anymore
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

            std::vector<Swath> find_optimal(Algorithm type) {
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
                        // swaths = exhaustive_search();
                        swaths = exhaustive_search();
                        break;
                    }
                    default:
                        throw std::invalid_argument("Unsupported algorithm type.");
                        swaths = std::vector<Swath>();
                        break;
                }
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

            // Brute Force Implementation: Exhaustive Search
            std::vector<Swath> exhaustive_search() {
                // Vector to store the path as a sequence of Swath objects
                std::vector<Swath> path;

                // Set to keep track of visited swaths to ensure each swath is traversed only once
                std::unordered_set<std::string> visited_swaths;

                // Reference to the map that associates points with their corresponding vertex in the graph
                auto& point_vertex_map = mesh_.point_vertex_map_;

                // Find the starting vertex in the mesh graph using the start_point_
                auto start_vertex_it = point_vertex_map.find(start_point_);
                if (start_vertex_it == point_vertex_map.end()) {
                    throw std::runtime_error("Start point not found in the mesh graph.");
                }
                auto start_vertex = start_vertex_it->second;

                // Find the ending vertex in the mesh graph using the end_point_
                auto end_vertex_it = point_vertex_map.find(end_point_);
                if (end_vertex_it == point_vertex_map.end()) {
                    throw std::runtime_error("End point not found in the mesh graph.");
                }
                auto end_vertex = end_vertex_it->second;

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

                // Initialize an invalid vertex descriptor to track the previous vertex (no previous vertex initially)
                boost::graph_traits<Mesh::Graph>::vertex_descriptor invalid_vertex = boost::graph_traits<Mesh::Graph>::null_vertex();

                // Start the recursive exhaustive search from the start_vertex
                bool found = recursive_ex(
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

            // Recursive helper function for exhaustive search using backtracking
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

                // Iterate over all outgoing edges from the current_vertex
                boost::graph_traits<Mesh::Graph>::out_edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::out_edges(current_vertex, mesh_.graph_); ei != ei_end; ++ei) {
                    auto edge = *ei;
                    const EdgeProperties& props = mesh_.graph_[edge]; // Retrieve properties of the edge
                    std::string swath_uuid = props.swath.uuid;  // Get the UUID of the swath associated with this edge

                    // Determine if the edge is a swath (of type LINE)
                    SwathType edge_type = props.swath.type;
                    bool is_swath = (edge_type == SwathType::LINE);

                    // If the edge is a swath and has already been visited, skip it
                    if (is_swath) {
                        if (visited_swaths.find(swath_uuid) != visited_swaths.end()) {
                            // std::cout << "Swath UUID " << swath_uuid << " already visited. Skipping.\n";
                            continue; // Skip this edge as it's already been traversed
                        }
                    }

                    // Identify the target vertex of the edge
                    auto target_vertex = boost::target(edge, mesh_.graph_);

                    // Prevent immediate backtracking to the previous vertex via a non-swath edge (e.g., a turn)
                    if (!is_swath && target_vertex == previous_vertex) {
                        // std::cout << "Skipping traversal back to previous vertex via turn.\n";
                        continue; // Skip to avoid redundant traversal
                    }

                    // Choose to traverse this edge
                    if (is_swath) {
                        // Mark the swath as visited and add it to the current path
                        visited_swaths.insert(swath_uuid);
                        path.push_back(props.swath); // Add Swath object directly
                        // std::cout << "Traversing through Line UUID: " << swath_uuid << "\n";
                    } else {
                        // For non-swath edges (e.g., turns), simply log the traversal
                        path.push_back(props.swath); // Add Swath object directly
                        // std::cout << "Traversing through Turn UUID: " << swath_uuid << "\n";
                    }

                    // Recurse with the updated current vertex and previous vertex
                    bool found = recursive_ex(
                        target_vertex,      // Move to the target vertex
                        end_vertex,         // Destination remains the same
                        visited_swaths,    // Updated set of visited swaths
                        path,               // Updated path
                        total_swaths,      // Total swaths to visit
                        current_vertex      // Current vertex becomes the previous vertex for the next recursion
                    );

                    if (found) {
                        return true; // Propagate the success up the recursion stack
                    }

                    // Backtrack: If the path wasn't successful, undo the traversal
                    if (is_swath) {
                        visited_swaths.erase(swath_uuid); // Unmark the swath as visited
                        path.pop_back();                   // Remove the swath from the current path
                        // std::cout << "Backtracking from swath UUID: " << swath_uuid << "\n";
                    }
                }

                return false; // No valid path found from this vertex
            }
    };
} // namespace farmtrax

#endif // PATH_HPP
