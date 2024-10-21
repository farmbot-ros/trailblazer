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

namespace farmtrax {
    class Path {
        public:
            // Enum to define different algorithm types
            enum class AlgorithmType {
                ASTAR,
                BFS,
            };

            // Constructor taking a reference to a Mesh object
            Path(const Mesh& mesh) : mesh_(mesh), start_set_(false), end_set_(false) {}

            // Method to set the initial and final points
            void set_points(const Point& start, const Point& end) {
                start_point_ = start;
                end_point_ = end;
                start_set_ = true;
                end_set_ = true;
            }

            std::vector<std::string> find_optimal(AlgorithmType type) {
                if (!start_set_ || !end_set_) {
                    throw std::runtime_error("Start and end points must be set before finding a path.");
                }
                switch (type) {
                    case AlgorithmType::ASTAR: {
                        return astar();
                    }
                    case AlgorithmType::BFS: {
                        return bfs();
                    }
                    default:
                        throw std::invalid_argument("Unsupported algorithm type.");
                }
            }

        private:
            const Mesh& mesh_;
            Point start_point_;
            Point end_point_;
            bool start_set_;
            bool end_set_;
        
            std::vector<std::string> astar() {
                std::vector<std::string> path;
                return path;
            }

            std::vector<std::string> bfs() {
                std::vector<std::string> path;
                return path;
            }
    };
} // namespace farmtrax
#endif // PATH_HPP
