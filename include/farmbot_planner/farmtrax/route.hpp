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
            std::vector<Swath> swaths;

        public:
            // Enum to define different algorithm types
            enum class Algorithm {
                A_STAR,
                EXHAUSTIVE_SEARCH,
                BREADTH_FIRST_SEARCH,
                DEPTH_FIRS_SEARCH,
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
                    case Algorithm::DEPTH_FIRS_SEARCH: {
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
                return swaths;
            }

            std::string print_path() const {
                std::string path_str;
                for (const auto& swath : swaths) {
                    path_str += swath.uuid + " -> ";
                }
                return path_str;
            }

        private:
            std::vector<Swath> uuid_2_swaths(const std::vector<std::string>& path_uuids) const {
                std::vector<Swath> swath_path;
                for (const auto& uuid : path_uuids) {
                    auto it = mesh_.uuid_to_swath_.find(uuid);
                    if (it != mesh_.uuid_to_swath_.end()) {
                        swath_path.push_back(it->second);
                    } else {
                        throw std::runtime_error("Swath UUID not found in mesh: " + uuid);
                    }
                }
                return swath_path;
            }

            // Brute Force Implementation
            std::vector<Swath> exhaustive_search() {
                std::vector<std::string> path;
                std::unordered_set<std::string> visited_swaths;

                // Initialize starting vertex
                auto& point_vertex_map = mesh_.point_vertex_map_;

                // Find the starting vertex
                auto start_vertex_it = point_vertex_map.find(start_point_);

                if (start_vertex_it == point_vertex_map.end()) {
                    throw std::runtime_error("Start point not found in the mesh graph.");
                }

                auto start_vertex = start_vertex_it->second;

                // Find the ending vertex
                auto end_vertex_it = point_vertex_map.find(end_point_);

                if (end_vertex_it == point_vertex_map.end()) {
                    throw std::runtime_error("End point not found in the mesh graph.");
                }

                auto end_vertex = end_vertex_it->second;

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
                bool found = recursive_ex(start_vertex, end_vertex, visited_swaths, path, total_swaths, invalid_vertex);

                if (found) {
                    std::cout << "Valid path found.\n";
                    return uuid_2_swaths(path);
                } else {
                    throw std::runtime_error("No valid path found that visits all swaths.");
                }
            }

            // Recursive helper function for brute force
            bool recursive_ex(
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

                    // Prevent immediate backtracking via turns
                    if (!is_swath && target_vertex == previous_vertex) {
                        std::cout << "Skipping traversal back to previous vertex via turn.\n";
                        continue;
                    }

                    // Choose to traverse this edge
                    if (is_swath) {
                        visited_swaths.insert(swath_uuid);
                        path.push_back(swath_uuid);
                        std::cout << "Traversing swath UUID: " << swath_uuid << "\n";
                    } else {
                        std::cout << "Traversing turn UUID: " << swath_uuid << "\n";
                    }

                    // Recurse with updated previous_vertex
                    bool found = recursive_ex(target_vertex, end_vertex, visited_swaths, path, total_swaths, current_vertex);
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

    };
} // namespace farmtrax

#endif // PATH_HPP
