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

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.

#include <cmath>
#include <map>
#include <queue>
#include <string>
#include <utility> // For std::pair
#include <vector>

namespace farmtrax {
    namespace bg = boost::geometry;
    typedef bg::model::d2::point_xy<double> Point;
    typedef bg::model::polygon<Point> Polygon;
    typedef bg::model::linestring<Point> LineString;
    typedef bg::model::box<Point> Box;
    typedef bg::model::multi_polygon<Polygon> Multipolygon;

    enum class SwathType { LINE, TURN, ROAD };

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

    class Swaths {
      private:
        std::vector<Swath> swaths_; // Holds Swath structs
        std::vector<Polygon> heardlands_;

      public:
        Swaths() = default;

        void gen_swaths(const Field &field, double swath_width, double angle_degrees, int number = 0) {
            // generate_swaths(field, swath_width, angle_degrees);
            Polygon fieldPolygon = field.get_polygon();
            if (number != 0) {
                heardlands_ = generate_headlands(swath_width, field.get_polygon(), number);
            }
            swaths_ = generate_swaths(fieldPolygon, swath_width, angle_degrees);
            // gen_headlands(swath_width, field.get_polygon(), number);
        }

        // Get the swaths as a vector of Swath structs
        const std::vector<Swath> &get_swaths() const { return swaths_; }

        // gange the swath order from last to first
        void reverse_swaths() { std::reverse(swaths_.begin(), swaths_.end()); }

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
                remove_colinear_points(simplifiedPolygon, 0.0001);
                polygon_array.push_back(simplifiedPolygon);
            }
            return polygon_array;
        }

        // Helper function to generate swaths with a specified angle
        std::vector<Swath> generate_swaths(Polygon &polygon, double swath_width, double angle_degrees) {
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
                }
            }
            return swaths;
        }

      private:
        // Function to generate a line at a certain offset from the center, adjusted for the angle
        LineString generate_swathine(const Point &centerPoint, double angle_radians, double offset,
                                     double length) const {
            LineString swathLine;

            // Calculate the perpendicular offset direction based on the angle
            double cos_angle = std::cos(angle_radians);
            double sin_angle = std::sin(angle_radians);

            // Calculate the start and end points of the swath line
            Point newStart(centerPoint.x() + (offset * sin_angle) - (length * cos_angle),
                           centerPoint.y() - (offset * cos_angle) - (length * sin_angle));

            Point newEnd(centerPoint.x() + (offset * sin_angle) + (length * cos_angle),
                         centerPoint.y() - (offset * cos_angle) + (length * sin_angle));

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
