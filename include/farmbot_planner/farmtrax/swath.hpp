#ifndef SWATH_HPP
#define SWATH_HPP

#include "field.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <boost/geometry/algorithms/distance.hpp> // For calculating distance

// Include Boost libraries for graph and R-tree
#include <boost/geometry/index/rtree.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/connected_components.hpp>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.

#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <queue>
#include <utility> // For std::pair

namespace farmtrax {
    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;

    // Define Cartesian point type
    typedef bg::model::d2::point_xy<double> Point;
    // Define polygon type using the Cartesian point
    typedef bg::model::polygon<Point> Polygon;
    // Define linestring type for edges
    typedef bg::model::linestring<Point> LineString;
    // Define box type
    typedef bg::model::box<Point> Box;
    // Define a multi-polygon type
    typedef bg::model::multi_polygon<Polygon> Multipolygon;

    typedef boost::adjacency_list<
        boost::vecS, 
        boost::vecS, 
        boost::undirectedS,
        boost::no_property, 
        boost::property<boost::edge_weight_t, double>> Graph;

    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef boost::graph_traits<Graph>::edge_descriptor Edge;

    // Enum class to represent different types of swaths
    enum class SwathType {
        LAND,
        HEAD,
        PATH
    };

    // Struct to represent each swath, along with its properties
    struct Swath {
        LineString swath;         // The actual swath line (geometry)
        std::string uuid;         // A unique identifier for each swath
        SwathType type;           // The type of swath (LAND, HEAD, PATH)
        bool transportlane;       // Flag to indicate if it's a transport lane
        float length;             // Length of the swath
    };

    // R-tree type definitions
    typedef std::pair<Box, std::size_t> RtreeValue;
    typedef bgi::rtree<RtreeValue, bgi::quadratic<16>> Rtree;

    class Swaths {
    private:
        Field inner_field_;
        Field outer_field_;
        double swath_width_;
        double angle_degrees_;
        std::vector<Swath> swaths_;  // Holds Swath structs
        Rtree swath_rtree_;          // R-tree for efficient spatial querying of swaths
        Graph swath_graph_;          // Graph to represent the swaths

    public:
        Swaths() = default;

        // Constructor to initialize with a field and swath width
        Swaths(const Field& outer_field, const Field& inner_field, double swath_width, double angle_degrees) {
            genSwaths(outer_field, inner_field, swath_width, angle_degrees);
        }

        void genSwaths(const Field& outer_field, const Field& inner_field, double swath_width, double angle_degrees) {
            outer_field_ = outer_field;
            inner_field_ = inner_field;
            swath_width_ = swath_width;
            angle_degrees_ = angle_degrees;
            generateSwaths();
        }

        // Get the swaths as a vector of Swath structs
        std::vector<Swath> getSwaths() const {
            return swaths_;
        }

    private:
        // Helper function to generate swaths with a specified angle
        void generateSwaths() {
            swaths_.clear();
            swath_rtree_.clear(); // Clear existing entries
            Field new_field = inner_field_.getBuffered(0.5, farmtrax::BufferType::ENLARGE);
            Polygon fieldPolygon = new_field.getPolygon();
            // Get the bounding box of the field
            boost::geometry::model::box<Point> boundingBox;
            boost::geometry::envelope(fieldPolygon, boundingBox);
            // Convert angle from degrees to radians
            double angle_radians = angle_degrees_ * M_PI / 180.0;
            // Determine the dimensions of the bounding box
            double width = boundingBox.max_corner().x() - boundingBox.min_corner().x();
            double height = boundingBox.max_corner().y() - boundingBox.min_corner().y();
            double max_dim = std::max(width, height);
            // Starting point and ending point adjusted for angle
            Point centerPoint((boundingBox.min_corner().x() + boundingBox.max_corner().x()) / 2,
                              (boundingBox.min_corner().y() + boundingBox.max_corner().y()) / 2);
            // Iterate to generate swaths with a defined offset based on swath width
            for (double offset = -max_dim / 2; offset <= max_dim / 2; offset += swath_width_) {
                LineString swathLine;
                // Calculate the perpendicular offset direction based on the angle
                double cos_angle = std::cos(angle_radians);
                double sin_angle = std::sin(angle_radians);
                // Define the half-length of the swath lines (to extend equally from the center)
                double half_length = 1000.0; // A large value to ensure swath lines extend beyond the field boundary
                // Calculate the start and end points of the swath line
                Point newStart(centerPoint.x() + (offset * sin_angle) - (half_length * cos_angle), centerPoint.y() - (offset * cos_angle) - (half_length * sin_angle));
                Point newEnd(centerPoint.x() + (offset * sin_angle) + (half_length * cos_angle), centerPoint.y() - (offset * cos_angle) + (half_length * sin_angle));
                // Add the new start and end points to the swath line
                swathLine.push_back(newStart);
                swathLine.push_back(newEnd);
                // Clip the swath line to fit within the field polygon
                std::vector<LineString> clipped;
                boost::geometry::intersection(swathLine, fieldPolygon, clipped);
                // Keep all valid segments of the swath that intersect the field polygon
                for (const auto& segment : clipped) {
                    Swath swath;  // Create a Swath struct for each segment
                    swath.swath = segment;
                    swath.uuid = boost::uuids::to_string(boost::uuids::random_generator()());  // Generate a unique ID for each swath
                    swath.type = SwathType::LAND; // Always mark as LAND here
                    swath.transportlane = false;  // Default, can modify as needed
                    swath.length = bg::length(segment); // Calculate the length of the swath
                    swaths_.push_back(swath);
                    // Add the vertex to the graph
                    boost::add_vertex(swath_graph_);
                    // Insert the swath into the R-tree
                    Box swath_box;
                    boost::geometry::envelope(segment, swath_box);
                    swath_rtree_.insert(std::make_pair(swath_box, swaths_.size() - 1));
                }
            }
            // Add edges with weights based on distance on the graph
            // Optimized to reduce complexity using R-tree
            double distance_threshold = swath_width_ * 2.0; // Example threshold, can be parameterized
            for (std::size_t i = 0; i < swaths_.size(); ++i) {
                // Query R-tree for swaths that are within the distance threshold
                std::vector<RtreeValue> nearby_swaths;
                Box query_box;
                boost::geometry::envelope(swaths_[i].swath, query_box);
                // Expand the query box by the distance threshold
                bg::expand(query_box, Box(Point(query_box.min_corner().x() - distance_threshold, query_box.min_corner().y() - distance_threshold),
                                        Point(query_box.max_corner().x() + distance_threshold, query_box.max_corner().y() + distance_threshold)));
                swath_rtree_.query(bgi::intersects(query_box), std::back_inserter(nearby_swaths));
                
                for (const auto& val : nearby_swaths) {
                    std::size_t j = val.second;
                    if (j > i) { // Ensure each edge is added only once
                        double distance = bg::distance(swaths_[i].swath, swaths_[j].swath);
                        if (distance <= distance_threshold) {
                            boost::add_edge(i, j, distance, swath_graph_);
                        }
                    }
                }
            }
        }
    };

} // namespace farmtrax

#endif // SWATH_HPP
