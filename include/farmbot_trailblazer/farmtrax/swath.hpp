#ifndef SWATH_HPP
#define SWATH_HPP

#include "field.hpp"
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.

#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <map>
#include <queue>
#include <string>
#include <utility> // For std::pair
#include <vector>

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

    // Enum class to represent different types of swaths
    enum class SwathType { LINE, TURN, ROAD };

    // Struct to represent each swath, along with its properties
    struct Swath {
        LineString swath; // The actual swath line (geometry)
        std::string uuid; // A unique identifier for each swath
        SwathType type;   // The type of swath (LINE, TURN, PATH)
        double length;    // Length of the swath

        bool intersects(const Field &field) const {
            Polygon fieldPolygon = field.get_polygon();
            return bg::intersects(fieldPolygon, swath);
        }

        void flip() {
            LineString reversed_swath = swath;
            std::reverse(reversed_swath.begin(), reversed_swath.end());
            swath = reversed_swath;
        }

        Swath create_swath(const Point &start, const Point &end, SwathType type, std::string uuid = "") {
            LineString line;
            bg::append(line, start);
            bg::append(line, end);
            std::string uuid_ = uuid.empty() ? boost::uuids::to_string(boost::uuids::random_generator()()) : uuid;
            return {line, uuid_, type, 0.0};
        }
    };

    // R-tree type definitions
    typedef std::pair<Box, std::size_t> RtreeValue;
    typedef bgi::rtree<RtreeValue, bgi::quadratic<16>> Rtree;

    class Swaths {
      private:
        std::vector<Swath> swaths_; // Holds Swath structs
        std::vector<Polygon> heardlands_;
        Rtree swath_rtree_; // R-tree for efficient spatial querying of swaths
        double colinear_threshold_ = 0.0001;

      public:
        Swaths() = default;

        // Constructor to initialize with a field and swath width
        // Swaths(const Field &field, double swath_width, double angle_degrees) {
        //     gen_swaths(field, swath_width, angle_degrees);
        // }

        void gen_swaths(const Field &field, double swath_width, double angle_degrees, int number = 1) {
            // generate_swaths(field, swath_width, angle_degrees);
            heardlands_ = generate_headlands(swath_width, field.get_polygon(), number);
            swaths_ = generate_swaths(heardlands_.back(), swath_width, angle_degrees);
            // gen_headlands(swath_width, field.get_polygon(), number);
        }

        // Get the swaths as a vector of Swath structs
        const std::vector<Swath> &get_swaths() const { return swaths_; }

        // Add a swath to the list of swaths
        void add_swath(const Swath &swath) { swaths_.push_back(swath); }

        // gange the swath order from last to first
        void reverse_swaths() { std::reverse(swaths_.begin(), swaths_.end()); }

        // get the heardlands
        const std::vector<Polygon> &get_heardlands() const { return heardlands_; }

        // If swath intersects with field
        bool intersects_field(const Field &field, const Swath &swath) {
            Polygon fieldPolygon = field.get_polygon();
            LineString swathLine = swath.swath;
            return boost::geometry::intersects(fieldPolygon, swathLine);
        }

        // Query swaths intersecting a given bounding box
        std::vector<std::size_t> query_swaths(const Box &query_box) const {
            std::vector<RtreeValue> result_s;
            swath_rtree_.query(bgi::intersects(query_box), std::back_inserter(result_s));

            std::vector<std::size_t> swath_indices;
            swath_indices.reserve(result_s.size());
            for (const auto &val : result_s) {
                swath_indices.push_back(val.second);
            }
            return swath_indices;
        }

        // Find the nearest swath to a given point
        std::size_t nearest_swath(const Point &point) const {
            std::vector<RtreeValue> result_s;
            swath_rtree_.query(bgi::nearest(point, 1), std::back_inserter(result_s));

            if (!result_s.empty()) {
                return result_s.front().second;
            } else {
                throw std::runtime_error("No swaths available.");
            }
        }

        // check if two points are connected by a swath
        bool are_connected(const Point &p1, const Point &p2) {
            LineString connection = create_connection(p1, p2);
            for (const auto &swath : swaths_) {
                if (bg::intersects(connection, swath.swath)) {
                    return true;
                }
            }
            return false;
        }

        // create a funtion that takes the generated swaths and divides them into nth goups
        std::vector<Swaths> divide_swaths(int n) {
            std::vector<Swaths> divided_swaths;
            std::vector<Swath> swaths = get_swaths();
            int num_swaths = swaths.size();
            int num_swaths_per_group = num_swaths / n;
            int remainder = num_swaths % n;
            int start = 0;
            int end = 0;
            for (int i = 0; i < n; i++) {
                end = start + num_swaths_per_group;
                if (remainder > 0) {
                    end++;
                    remainder--;
                }
                std::vector<Swath> group_swaths(swaths.begin() + start, swaths.begin() + end);
                Swaths group_swaths_obj;
                group_swaths_obj.swaths_ = group_swaths;
                divided_swaths.push_back(group_swaths_obj);
                start = end;
            }
            return divided_swaths;
        }

      private:
        std::vector<Polygon> generate_headlands(double x, Polygon polygon_, int number = 1) const {
            if (x < 0) {
                throw std::invalid_argument("Shrink distance must be non-negative.");
            }
            std::vector<Polygon> polygon_array;
            for (int i = 0; i < number; i++) {
                auto polygon = i == 0 ? polygon_ : polygon_array.back();
                // Define buffer strategies with straight edges
                bg::strategy::buffer::distance_symmetric<double> distance_strategy(-x);
                bg::strategy::buffer::side_straight side_strategy;
                bg::strategy::buffer::join_miter join_strategy;
                // bg::strategy::buffer::join_round join_strategy;
                bg::strategy::buffer::end_flat end_strategy;
                bg::strategy::buffer::point_square point_strategy;
                // Perform buffering with negative distance to shrink the polygon
                Multipolygon result;
                bg::buffer(polygon, result, distance_strategy, side_strategy, join_strategy, end_strategy,
                           point_strategy);
                if (result.empty()) {
                    throw std::runtime_error("Shrinking resulted in an empty field.");
                }
                // Select the largest polygon from the result
                const Polygon *largest = nullptr;
                double max_area = -std::numeric_limits<double>::max();
                for (const auto &poly : result) {
                    double area = bg::area(poly);
                    if (area > max_area) {
                        max_area = area;
                        largest = &poly;
                    }
                }
                if (!largest) {
                    throw std::runtime_error("Failed to determine the largest polygon after shrinking.");
                }
                Polygon simplifiedPolygon = *largest;
                remove_colinear_points(simplifiedPolygon, colinear_threshold_);
                polygon_array.push_back(simplifiedPolygon);
            }
            return polygon_array;
        }

        // Helper function to generate swaths with a specified angle
        std::vector<Swath> generate_swaths(Polygon &polygon, double swath_width, double angle_degrees) {
            swath_rtree_.clear(); // Clear existing entries
            std::vector<Swath> swaths;

            Field field = Field(polygon);
            Polygon fieldPolygon = field.get_polygon();

            // Get the **rotated bounding box**
            Polygon rotatedBoundingBox = get_rotated_bounding_box(fieldPolygon);

            // Convert angle from degrees to radians
            double angle_radians = angle_degrees * M_PI / 180.0;

            // Compute the center of the rotated bounding box
            Point centerPoint;
            boost::geometry::centroid(rotatedBoundingBox, centerPoint);

            // Get the maximum dimensions of the rotated bounding box
            auto &outer = rotatedBoundingBox.outer();
            double width = boost::geometry::distance(outer[0], outer[1]);  // Width of rotated box
            double height = boost::geometry::distance(outer[1], outer[2]); // Height of rotated box
            double max_dim = std::hypot(width, height);                    // Use diagonal length for full coverage

            // Iterate to generate swaths with a defined offset based on swath width
            auto new_polygon = fieldPolygon;
            for (double offset = -max_dim; offset <= max_dim; offset += swath_width) {
                double length = max_dim * 2; // Extend to ensure full coverage
                LineString swathLine = generate_swathine(centerPoint, angle_radians, offset, length);

                // Clip the swath line to fit within the field polygon
                std::vector<LineString> clipped;
                boost::geometry::intersection(swathLine, fieldPolygon, clipped);

                // Keep all valid segments of the swath that intersect the field polygon
                for (const auto &segment : clipped) {
                    if (boost::geometry::length(segment) < swath_width) {
                        continue; // Ignore very short swaths
                    }
                    Swath swath;
                    swath.swath = segment;
                    swath.uuid = generate_UUID(); // Generate unique ID
                    swath.type = SwathType::LINE;
                    swath.length = boost::geometry::length(segment);

                    swaths.push_back(swath);

                    insert_point_at_closest_location(new_polygon, segment.front());
                    insert_point_at_closest_location(new_polygon, segment.back());

                    // Insert the swath into the R-tree
                    Box swath_box;
                    boost::geometry::envelope(segment, swath_box);
                    swath_rtree_.insert(std::make_pair(swath_box, swaths_.size() - 1));
                }
            }
            return swaths;
        }

        // Function to generate a line at a certain offset from the center, adjusted for the angle
        LineString generate_swathine(const Point &center, double angle_radians, double offset, double length) const {
            LineString swathLine;

            // Calculate the perpendicular offset direction based on the angle
            double cos_angle = std::cos(angle_radians);
            double sin_angle = std::sin(angle_radians);

            // Calculate the start and end points of the swath line
            Point newStart(center.x() + (offset * sin_angle) - (length * cos_angle),
                           center.y() - (offset * cos_angle) - (length * sin_angle));

            Point newEnd(center.x() + (offset * sin_angle) + (length * cos_angle),
                         center.y() - (offset * cos_angle) + (length * sin_angle));

            // Add the new start and end points to the swath line
            swathLine.push_back(newStart);
            swathLine.push_back(newEnd);

            return swathLine;
        }

        void insert_point_at_closest_location(Polygon &poly, const Point &p) {
            auto &outer_ring = poly.outer();
            // Check if the ring is closed (first and last points are the same)
            bool is_closed = !outer_ring.empty() && bg::equals(outer_ring.front(), outer_ring.back());
            // Remove the closing point if the ring is closed
            if (is_closed) {
                outer_ring.pop_back();
            }
            // Variables to keep track of the closest segment
            double min_distance = std::numeric_limits<double>::max();
            size_t insert_position = 0; // Position to insert the point
            // Iterate over the segments of the outer ring
            for (size_t i = 0; i < outer_ring.size(); ++i) {
                // Get the current segment
                Point p1 = outer_ring[i];
                Point p2 = outer_ring[(i + 1) % outer_ring.size()]; // Wrap around for the last segment
                bg::model::segment<Point> seg(p1, p2);
                // Compute the distance from the point to the segment
                double distance = bg::distance(p, seg);
                // Update the minimum distance and insertion position if necessary
                if (distance < min_distance) {
                    min_distance = distance;
                    insert_position = i + 1; // Insert after point i
                }
            }
            // Insert the point at the determined position
            outer_ring.insert(outer_ring.begin() + insert_position, p);
            // Close the ring by adding the first point at the end
            if (outer_ring.size() >= 3) {
                outer_ring.push_back(outer_ring.front());
            }
            // Optional: Correct the polygon to ensure validity (orientation, closure)
            bg::correct(poly);
            // Check if the polygon is valid
            if (!bg::is_valid(poly)) {
                throw std::runtime_error("Polygon is invalid after insertion.");
            }
        }

        // Function to create a connection between two points
        LineString create_connection(const Point &p1, const Point &p2) const {
            LineString connection;
            connection.push_back(p1);
            connection.push_back(p2);
            return connection;
        }

        // Function to generate a unique identifier for each swath
        std::string generate_UUID() const { return boost::uuids::to_string(boost::uuids::random_generator()()); }

        // function that cheks if swath touches perimeter of the field
        bool intersects_field(const LineString &swath, const Field &field) {
            auto edges = field.get_edges();
            for (const auto &edge : edges) {
                if (bg::intersects(swath, edge)) {
                    return true;
                }
            }
            return false;
        }

        Polygon get_rotated_bounding_box(const Polygon &polygon) {
            Polygon hullPolygon;
            boost::geometry::convex_hull(polygon, hullPolygon);
            boost::geometry::correct(hullPolygon); // Ensure a valid polygon
            return hullPolygon;
        }
    };

} // namespace farmtrax

#endif // SWATH_HPP
