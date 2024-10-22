#ifndef MESH_HPP
#define MESH_HPP

#include "field.hpp"
#include "swath.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <vector>
#include <string>
#include <map>
#include <set>
#include <limits>
#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <stack>
#include <sstream>      // Added for std::ostringstream
#include <fstream>      // Added for std::ofstream

namespace farmtrax {
    namespace bg = boost::geometry;
    // Define Cartesian point type
    typedef bg::model::d2::point_xy<double> Point;
    // Define linestring type for edges
    typedef bg::model::linestring<Point> LineString;
    // Polygon type for field boundary
    typedef bg::model::polygon<Point> Polygon;


    // Edge property to indicate if the edge is bidirectional (undirected), its type, and its weight
    struct EdgeProperties {
        Swath swath;                    // Swath object
        double weight;                  // Weight of the edge (for algorithms)

        EdgeProperties() = default;
        EdgeProperties(const Swath& swath, double weight) : swath(swath), weight(weight) {}
    };

    struct VertexProperties {
        Point point;

        //comparison operator for vertices
        bool operator==(const VertexProperties& rhs) const {
            //user a tolerance for floating-point comparisons
            const double epsilon = 0.01;
            return (std::abs(point.x() - rhs.point.x()) < epsilon) && (std::abs(point.y() - rhs.point.y()) < epsilon);
        }

        VertexProperties() = default;
        VertexProperties(const Point& point) : point(point) {}
    };

    // Comparator for Point
    struct PointComp {
        bool operator()(const Point& lhs, const Point& rhs) const {
            // Use a tolerance for floating-point comparisons
            const double epsilon = 0.01;

            if (std::abs(lhs.x() - rhs.x()) > epsilon) {
                return lhs.x() < rhs.x();
            } else {
                return lhs.y() < rhs.y();
            }
        }
    };

    class Mesh {
        private:
            Swaths swaths_;

        public:
            // Define the graph type with VertexProperties and EdgeProperties
            typedef boost::adjacency_list<
                boost::vecS,                   // How to store vertices
                boost::vecS,                   // How to store edges
                boost::directedS,              // Directed graph
                VertexProperties,              // Vertex properties
                EdgeProperties                 // Edge properties
            > Graph;

            Graph graph_;
            // Map to store unique vertices based on their points
            std::map<Point, boost::graph_traits<Graph>::vertex_descriptor, PointComp> point_vertex_map_;
            // List of required edges (swaths)
            std::vector<boost::graph_traits<Graph>::edge_descriptor> required_edges_;

        public:
            // Default constructor
            Mesh() = default;

            // Constructor that initializes the route with swaths and builds the graph
            Mesh(const Swaths& swaths){
                swaths_ = swaths;
            }

            // Function to build the mixed graph
            void build_graph(bool weight_on_headlands = false) {
                // Clear any existing graph data
                graph_ = Graph();
                point_vertex_map_.clear();
                required_edges_.clear();

                // Lambda to get or create a vertex
                auto get_or_create_vertex = [&](const Point& p) -> boost::graph_traits<Graph>::vertex_descriptor {
                    auto it = point_vertex_map_.find(p);
                    if (it != point_vertex_map_.end()) {
                        return it->second;
                    } else {
                        boost::graph_traits<Graph>::vertex_descriptor v = boost::add_vertex(graph_);
                        graph_[v].point = p;  // Set the vertex property
                        point_vertex_map_[p] = v;
                        return v;
                    }
                };

                // Iterate over all swaths and headlands
                for (const auto& swath : swaths_.get_swaths_with_headland()) {
                    const Point& start_point = swath.swath.front();
                    const Point& end_point = swath.swath.back();

                    boost::graph_traits<Graph>::vertex_descriptor start_vertex = get_or_create_vertex(start_point);
                    boost::graph_traits<Graph>::vertex_descriptor end_vertex = get_or_create_vertex(end_point);

                    if (swath.type == SwathType::LINE) {
                        // Swaths are directed edges
                        EdgeProperties props(swath, weight_on_headlands ? 1.0 : 0.0);

                        boost::graph_traits<Graph>::edge_descriptor e;
                        bool inserted;
                        boost::tie(e, inserted) = boost::add_edge(start_vertex, end_vertex, props, graph_);
                        if (inserted) {
                            required_edges_.push_back(e); // Mark as required
                        }
                    }
                    else {
                        // Headlands are undirected edges (add two directed edges)
                        EdgeProperties props_forward(swath, weight_on_headlands ? 0.0 : 1.0);
                        EdgeProperties props_reverse(swath.reverse(), weight_on_headlands ? 0.0 : 1.0);
                        // EdgeProperties props_forward(swath.uuid, SwathType::TURN, weight_on_headlands ? 0.0 : 1.0);
                        // EdgeProperties props_reverse(swath.uuid, SwathType::TURN, weight_on_headlands ? 0.0 : 1.0);

                        boost::graph_traits<Graph>::edge_descriptor e1, e2;
                        bool inserted1, inserted2;
                        boost::tie(e1, inserted1) = boost::add_edge(start_vertex, end_vertex, props_forward, graph_);
                        boost::tie(e2, inserted2) = boost::add_edge(end_vertex, start_vertex, props_reverse, graph_);
                        // Headlands are optional, not required to be traversed
                    }
                }
            }

            // Function to get the string representation of the graph
            std::string to_string() const {
                std::ostringstream oss;
                oss << "Vertices: " << boost::num_vertices(graph_) << std::endl;
                oss << "Edges: " << boost::num_edges(graph_) << std::endl;

                // Iterate over all vertices
                boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
                for (boost::tie(vi, vi_end) = boost::vertices(graph_); vi != vi_end; ++vi) {
                    boost::graph_traits<Graph>::vertex_descriptor v = *vi;
                    const Point& p = graph_[v].point;
                    oss << "Vertex " << v << ": (" << p.x() << ", " << p.y() << ")" << std::endl;
                }

                // Iterate over all edges
                boost::graph_traits<Graph>::edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei) {
                    boost::graph_traits<Graph>::edge_descriptor e = *ei;
                    boost::graph_traits<Graph>::vertex_descriptor s = boost::source(e, graph_);
                    boost::graph_traits<Graph>::vertex_descriptor t = boost::target(e, graph_);
                    const Point& sp = graph_[s].point;
                    const Point& tp = graph_[t].point;
                    const EdgeProperties& props = graph_[e];
                    oss << "Edge " << e << ": (" << sp.x() << ", " << sp.y() << ") -> ("
                        << tp.x() << ", " << tp.y() << ")";
                    oss << "("
                        << "swath: " << props.swath.uuid
                        << ", type: " << static_cast<int>(props.swath.type)
                        << ", weight: " << props.weight
                        << ")" << std::endl;
                }

                return oss.str();
            }

            // Create Graphviz DOT file
            std::string to_dot(const std::string& filename = "") const {
                std::ostringstream oss;
                oss << "digraph G {" << std::endl;

                // Iterate over all vertices
                boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
                for (boost::tie(vi, vi_end) = boost::vertices(graph_); vi != vi_end; ++vi) {
                    boost::graph_traits<Graph>::vertex_descriptor v = *vi;
                    const Point& p = graph_[v].point;
                    oss << "  " << v << " [label=\"(" << p.x() << ", " << p.y() << ")\"];" << std::endl;
                }

                // Iterate over all edges
                boost::graph_traits<Graph>::edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei) {
                    boost::graph_traits<Graph>::edge_descriptor e = *ei;
                    boost::graph_traits<Graph>::vertex_descriptor s = boost::source(e, graph_);
                    boost::graph_traits<Graph>::vertex_descriptor t = boost::target(e, graph_);
                    // Point sp = graph_[s].point;
                    // Point tp = graph_[t].point;
                    const EdgeProperties& props = graph_[e];
                    oss << "  " << s << " -> " << t << " [label=\""
                        << props.swath.uuid << " (" << props.weight << ", "
                        << (props.swath.type == SwathType::LINE ? "LINE" : "TURN")
                        << ")\"];" << std::endl;
                }
                oss << "}" << std::endl;
                if (!filename.empty()) {
                    std::ofstream ofs(filename);
                    if (ofs) {
                        ofs << oss.str();
                    } else {
                        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
                    }
                }
                return oss.str();
            }

        private:
            // Helper function remains unchanged
        };

} // namespace farmtrax

#endif // MESH_HPP
