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


#include "rclcpp/rclcpp.hpp"

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
#include "spdlog/spdlog.h"
namespace echo = spdlog;

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
        VertexProperties() = default;
        VertexProperties(const Point& point) : point(point) {}
    };

    class Mesh {
        private:
            rclcpp::Node::SharedPtr node_;          // ROS 2 node handle
            std::vector<Swath> swaths_;             // List of swaths

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

        public:
            // Default constructor
            Mesh() = default;

            void pass_node(rclcpp::Node::SharedPtr node) {
                node_ = node;
            }

            void build_graph(std::vector<Swath> swaths, bool weight_on_headlands = false) {
                swaths_ = swaths;
                // Clear any existing graph data
                graph_ = Graph();

                // Lambda to find a vertex based on a point
                auto find_vertex_by_point = [&](const Point& p) -> boost::graph_traits<Graph>::vertex_descriptor {
                    boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
                    for (boost::tie(vi, vi_end) = boost::vertices(graph_); vi != vi_end; ++vi) {
                        if (bg::equals(graph_[*vi].point, p)) {
                            return *vi;
                        }
                    }
                    return boost::graph_traits<Graph>::null_vertex();
                };

                // Lambda to get or create a vertex
                auto get_or_create_vertex = [&](const Point& p) -> boost::graph_traits<Graph>::vertex_descriptor {
                    boost::graph_traits<Graph>::vertex_descriptor v = find_vertex_by_point(p);
                    if (v != boost::graph_traits<Graph>::null_vertex()) {
                        return v;
                    } else {
                        v = boost::add_vertex(graph_);
                        graph_[v].point = p;  // Set the vertex property
                        return v;
                    }
                };

                // Lambda to check if an edge exists between two vertices
                auto edge_exists_between_vertices = [&](boost::graph_traits<Graph>::vertex_descriptor v1, boost::graph_traits<Graph>::vertex_descriptor v2) -> bool {
                    boost::graph_traits<Graph>::edge_descriptor e;
                    bool exists;
                    boost::tie(e, exists) = boost::edge(v1, v2, graph_);
                    return exists;
                };

                // First, add edges for each swath from its start to end point
                for (const auto& swath : swaths_) {
                    const Point& start_point = swath.swath.front();
                    const Point& end_point = swath.swath.back();

                    boost::graph_traits<Graph>::vertex_descriptor start_vertex = get_or_create_vertex(start_point);
                    boost::graph_traits<Graph>::vertex_descriptor end_vertex = get_or_create_vertex(end_point);

                    // Check if edge already exists between start_vertex and end_vertex
                    if (!edge_exists_between_vertices(start_vertex, end_vertex)) {
                        double weight = 0.0;
                        EdgeProperties props(swath, weight);
                        boost::add_edge(start_vertex, end_vertex, props, graph_);
                    }
                }

                // Now, connect each swath's end to every other swath's start (excluding itself)
                const auto& swaths_list = swaths_;
                for (const auto& swath_i : swaths_list) {
                    const Point& end_point_i = swath_i.swath.back();
                    boost::graph_traits<Graph>::vertex_descriptor end_vertex_i = get_or_create_vertex(end_point_i);

                    for (const auto& swath_j : swaths_list) {
                        if (swath_i.uuid == swath_j.uuid) {
                            continue; // Skip if it's the same swath
                        }
                        const Point& start_point_j = swath_j.swath.front();
                        boost::graph_traits<Graph>::vertex_descriptor start_vertex_j = get_or_create_vertex(start_point_j);

                        // Check if edge from end_vertex_i to start_vertex_j already exists
                        if (!edge_exists_between_vertices(end_vertex_i, start_vertex_j)) {
                            // Calculate weight (e.g., Euclidean distance between end_point_i and start_point_j)
                            double weight = bg::distance(end_point_i, start_point_j);
                            // Create a connection swath representing the turn (from end of swath_i to start of swath_j)
                            Swath connection_swath;
                            connection_swath.type = SwathType::TURN; // Assuming TURN is the type for these connections
                            connection_swath.swath.push_back(end_point_i);
                            connection_swath.swath.push_back(start_point_j);
                            connection_swath.uuid = boost::uuids::to_string(boost::uuids::random_generator()());

                            EdgeProperties props(connection_swath, weight);

                            // Add the edge
                            boost::add_edge(end_vertex_i, start_vertex_j, props, graph_);
                        }
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

            //function to get the string representation of the graph, each vertex and then tabbed edges
            std::string to_string2() const {
                std::ostringstream oss;
                oss << "Vertices: " << boost::num_vertices(graph_) << std::endl;
                oss << "Edges: " << boost::num_edges(graph_) << std::endl;

                // Iterate over all vertices
                boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
                for (boost::tie(vi, vi_end) = boost::vertices(graph_); vi != vi_end; ++vi) {
                    boost::graph_traits<Graph>::vertex_descriptor v = *vi;
                    const Point& p = graph_[v].point;
                    oss << "Vertex " << v << ": (" << p.x() << ", " << p.y() << ")" << std::endl;
                    // Iterate over all edges
                    boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
                    for (boost::tie(ei, ei_end) = boost::out_edges(v, graph_); ei != ei_end; ++ei) {
                        boost::graph_traits<Graph>::edge_descriptor e = *ei;
                        boost::graph_traits<Graph>::vertex_descriptor t = boost::target(e, graph_);
                        const Point& tp = graph_[t].point;
                        const EdgeProperties& props = graph_[e];
                        oss << "\tEdge " << e << ": (" << p.x() << ", " << p.y() << ") -> ("
                            << tp.x() << ", " << tp.y() << ")";
                        oss << "("
                            << "swath: " << props.swath.uuid
                            << ", type: " << static_cast<int>(props.swath.type)
                            << ", weight: " << props.weight
                            << ")" << std::endl;
                    }
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
