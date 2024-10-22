#ifndef PATH_HPP
#define PATH_HPP

#include "mesh.hpp"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/depth_first_search.hpp>
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
    namespace bg = boost::geometry; // Ensure namespace alias is defined

    class Route {
        private:
            Mesh mesh_;
            Point start_point_;
            Point end_point_;
            bool start_set_;
            bool end_set_;

            std::vector<std::string> uuids;

        public:
            // Enum to define different algorithm types
            enum class Algorithm {
                A_STAR,
                BREATH_FIRST_SEARCH,
                BRUTE_FORCE,
                EXHAUSTIVE_SEARCH,
            };

            // Default constructor
            Route() : mesh_(), start_set_(false), end_set_(false) {}

            // Constructor that initializes with a Mesh instance
            Route(Mesh mesh) : mesh_(mesh), start_set_(false), end_set_(false) {}

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
                const auto& vertex_point_map = mesh_.vertex_point_map_;
                // Example Strategy 1: Choose the first and last vertices in the map
                if (!start_set_) {
                    if (vertex_point_map.empty()) {
                        throw std::runtime_error("Mesh graph has no vertices to select as start point.");
                    }
                    auto it = vertex_point_map.begin();
                    start_point_ = it->second;
                    start_set_ = true;
                    std::cout << "Default start point selected: (" << start_point_.x() << ", " << start_point_.y() << ")\n";
                }

                if (!end_set_) {
                    if (vertex_point_map.empty()) {
                        throw std::runtime_error("Mesh graph has no vertices to select as end point.");
                    }
                    auto it = vertex_point_map.end();
                    --it;
                    end_point_ = it->second;
                    end_set_ = true;
                    std::cout << "Default end point selected: (" << end_point_.x() << ", " << end_point_.y() << ")\n";
                }

                // Alternatively, implement another strategy, such as selecting vertices with min/max coordinates
                // Uncomment the following lines to use the min/max coordinate strategy

                /*
                // Strategy 2: Select vertices with minimum and maximum x + y coordinates
                auto min_max = std::minmax_element(vertex_point_map.begin(), vertex_point_map.end(),
                    [](const std::pair<boost::graph_traits<Mesh::Graph>::vertex_descriptor, Point>& a,
                    const std::pair<boost::graph_traits<Mesh::Graph>::vertex_descriptor, Point>& b) -> bool {
                        return (a.second.x() + a.second.y()) < (b.second.x() + b.second.y());
                    });
                start_point_ = min_max.first->second;
                end_point_ = min_max.second->second;
                start_set_ = true;
                end_set_ = true;
                */
            }

            std::vector<Swath> find_optimal(Algorithm type) {
                select_default_points(); // Set start and end points
                switch (type) {
                    case Algorithm::A_STAR: {
                        // Implementation for A* can be added later
                        uuids =  a_star();
                        break;
                    }
                    case Algorithm::BREATH_FIRST_SEARCH: {
                        // Implementation for BFS can be added later
                        uuids = breath_first_search();
                        break;
                    }
                    case Algorithm::BRUTE_FORCE: {
                        uuids = brute_force();
                        break;
                    }
                    case Algorithm::EXHAUSTIVE_SEARCH: {
                        uuids = exhaustive_search();
                        break;
                    }
                    default:
                        // throw std::invalid_argument("Unsupported algorithm type.");
                        uuids = std::vector<std::string>();
                        break;
                }
                std::vector<Swath> swath_path;
                for (const auto& uuid : uuids) {
                    auto it = mesh_.uuid_to_swath_.find(uuid);
                    if (it != mesh_.uuid_to_swath_.end()) {
                        swath_path.push_back(it->second);
                    } else {
                        throw std::runtime_error("Swath UUID not found in mesh: " + uuid);
                    }
                }
                return swath_path;

            }

            std::string print_path() const {
                std::string path_str;
                for (const auto& uuid : uuids) {
                    path_str += uuid + " -> ";
                }
                return path_str;
            }

        private:
            // Brute Force Implementation
            std::vector<std::string> brute_force() {
                std::vector<std::string> path;
                std::unordered_set<std::string> visited_swaths;

                // Initialize starting vertex
                auto graph = mesh_.graph_;
                auto& vertex_point_map = mesh_.vertex_point_map_;

                // Find the starting vertex
                auto start_vertex_it = std::find_if(vertex_point_map.begin(), vertex_point_map.end(),
                    [&](const std::pair<boost::graph_traits<Mesh::Graph>::vertex_descriptor, Point>& pair) {
                        return bg::equals(pair.second, start_point_);
                    });

                if (start_vertex_it == vertex_point_map.end()) {
                    throw std::runtime_error("Start point not found in the mesh graph.");
                }

                auto start_vertex = start_vertex_it->first;

                // Find the ending vertex
                auto end_vertex_it = std::find_if(vertex_point_map.begin(), vertex_point_map.end(),
                    [&](const std::pair<boost::graph_traits<Mesh::Graph>::vertex_descriptor, Point>& pair) {
                        return bg::equals(pair.second, end_point_);
                    });

                if (end_vertex_it == vertex_point_map.end()) {
                    throw std::runtime_error("End point not found in the mesh graph.");
                }

                auto end_vertex = end_vertex_it->first;

                // Calculate total_swaths to include only swaths (SwathType::LINE)
                size_t total_swaths = std::count_if(
                    mesh_.uuid_to_swath_.begin(),
                    mesh_.uuid_to_swath_.end(),
                    [](const std::pair<std::string, Swath>& pair) {
                        return pair.second.type == SwathType::LINE;
                    }
                );

                std::cout << "Total swaths to visit: " << total_swaths << "\n";

                // Recursive backtracking to find the path
                // Initialize previous_vertex as invalid to indicate no previous vertex
                boost::graph_traits<Mesh::Graph>::vertex_descriptor invalid_vertex = boost::graph_traits<Mesh::Graph>::null_vertex();
                bool found = recursive_brute_force(start_vertex, end_vertex, visited_swaths, path, total_swaths, invalid_vertex);

                if (found) {
                    std::cout << "Valid path found.\n";
                    return path;
                } else {
                    throw std::runtime_error("No valid path found that visits all swaths.");
                }
            }

            // Recursive helper function for brute force
            bool recursive_brute_force(
                boost::graph_traits<Mesh::Graph>::vertex_descriptor current_vertex,
                boost::graph_traits<Mesh::Graph>::vertex_descriptor end_vertex,
                std::unordered_set<std::string>& visited_swaths,
                std::vector<std::string>& path,
                size_t total_swaths,
                boost::graph_traits<Mesh::Graph>::vertex_descriptor previous_vertex
            ) {
                // If current vertex is the end and all swaths are visited
                if (current_vertex == end_vertex && visited_swaths.size() == total_swaths) {
                    return true;
                }

                // Iterate over all outgoing edges from current_vertex
                boost::graph_traits<Mesh::Graph>::out_edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::out_edges(current_vertex, mesh_.graph_); ei != ei_end; ++ei) {
                    std::cout << "Visiting edge (" << boost::source(*ei, mesh_.graph_) << "," << boost::target(*ei, mesh_.graph_) << ")\n";
                    auto edge = *ei;
                    EdgeProperties props = mesh_.graph_[edge];
                    std::string swath_uuid = props.swath_uuid;

                    // Determine the type of the edge
                    SwathType edge_type = props.type;

                    bool is_swath = (edge_type == SwathType::LINE);

                    if (is_swath) {
                        if (visited_swaths.find(swath_uuid) != visited_swaths.end()) {
                            std::cout << "Swath UUID " << swath_uuid << " already visited. Skipping.\n";
                            continue; // Already visited this swath
                        }
                    }

                    // Move to the target vertex
                    auto target_vertex = boost::target(edge, mesh_.graph_);

                    // Prevent immediate backtracking via headlands
                    if (!is_swath && target_vertex == previous_vertex) {
                        std::cout << "Skipping traversal back to previous vertex via headland.\n";
                        continue;
                    }

                    // Choose to traverse this edge
                    if (is_swath) {
                        visited_swaths.insert(swath_uuid);
                        path.push_back(swath_uuid);
                        std::cout << "Traversing swath UUID: " << swath_uuid << "\n";
                    } else {
                        std::cout << "Traversing headland UUID: " << swath_uuid << "\n";
                    }

                    // Recurse with updated previous_vertex
                    bool found = recursive_brute_force(target_vertex, end_vertex, visited_swaths, path, total_swaths, current_vertex);
                    if (found) {
                        return true;
                    }

                    // Backtrack
                    if (is_swath) {
                        visited_swaths.erase(swath_uuid);
                        path.pop_back();
                        std::cout << "Backtracking from swath UUID: " << swath_uuid << "\n";
                    }
                }

                return false; // No valid path found from this vertex
            }

            // Placeholder for A* algorithm
            std::vector<std::string> a_star() {
                std::vector<std::string> path;
                // Implementation for A* can be added here
                return path;
            }

            // Placeholder for Breadth First Search algorithm
            std::vector<std::string> breath_first_search() {
                std::vector<std::string> path;
                // Implementation for BFS can be added here
                return path;
            }

            // Placeholder for exhaustive search algorithm
            std::vector<std::string> exhaustive_search() {
                std::vector<std::string> path;
                // Implementation for exhaustive search can be added here
                return path;
            }
    };
} // namespace farmtrax

#endif // PATH_HPP
