#ifndef LINE_HPP
#define LINE_HPP

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
#include <string>
#include <vector>

namespace farmtrax {
    namespace bg = boost::geometry;
    typedef bg::model::d2::point_xy<double> Point;
    typedef bg::model::polygon<Point> Polygon;
    typedef bg::model::linestring<Point> LineString;
    typedef bg::model::box<Point> Box;
    typedef bg::model::multi_polygon<Polygon> Multipolygon;

    // Function to check if three points are colinear
    template <typename Point>
    bool are_colinear(const Point &p1, const Point &p2, const Point &p3, double epsilon = 1e-10) {
        // Calculate the area of the triangle formed by the three points
        // If the area is zero (or close to zero), the points are colinear

        auto area = (boost::geometry::get<0>(p1) * (boost::geometry::get<1>(p2) - boost::geometry::get<1>(p3)) +
                     boost::geometry::get<0>(p2) * (boost::geometry::get<1>(p3) - boost::geometry::get<1>(p1)) +
                     boost::geometry::get<0>(p3) * (boost::geometry::get<1>(p1) - boost::geometry::get<1>(p2))) /
                    2.0;

        return std::abs(area) < epsilon;
    }

    // Function to remove colinear points from a polygon
    template <typename Polygon> void remove_colinear_points(Polygon &polygon, double epsilon = 0.01) {
        using point_type = typename boost::geometry::point_type<Polygon>::type;
        std::vector<point_type> new_points;
        auto const &points = polygon.outer();

        if (points.size() < 4) return; // Need at least 3 distinct points (excluding duplicate endpoint)

        // Since the polygon is closed, the first and last points are the same.
        // We iterate excluding the duplicate last point.
        size_t n = points.size() - 1;
        for (size_t i = 0; i < n; ++i) {
            const point_type &prev = points[(i + n - 1) % n];
            const point_type &curr = points[i];
            const point_type &next = points[(i + 1) % n];

            if (!are_colinear(prev, curr, next, epsilon)) {
                new_points.push_back(curr);
            }
            // If colinear, skip the current point
        }

        // Close the polygon by adding the first point at the end
        if (!new_points.empty()) {
            new_points.push_back(new_points.front());
        }

        polygon.outer().assign(new_points.begin(), new_points.end());
    }

    // Struct to represent a headland
    struct Ring {
        Polygon polygon;
        std::string uuid;
    };

    // Enum to define different swath types
    enum class SwathType { LINE, TURN, ROAD };

    // Struct to represent a swath
    struct Swath {
        LineString line;  // The actual swath line (geometry)
        std::string uuid; // A unique identifier for each swath
        SwathType type;   // The type of swath (LINE, TURN, PATH)
        double length;    // Length of the swath

        void flip() {
            LineString reversed_swath = line;
            std::reverse(reversed_swath.begin(), reversed_swath.end());
            line = reversed_swath;
        }
    };

    Swath inline create_swath(const Point &start, const Point &end, SwathType type, std::string uuid = "") {
        LineString line;
        bg::append(line, start);
        bg::append(line, end);
        std::string uuid_ = uuid.empty() ? boost::uuids::to_string(boost::uuids::random_generator()()) : uuid;
        return {line, uuid_, type, 0.0};
    }

    class Field {
      private:
        Polygon border_;
        std::vector<Swath> swaths_;
        std::vector<Ring> headlands_;

      public:
        Field() = default;

        void gen_border(const std::vector<std::vector<double>> &coordinates) {
            std::vector<std::pair<double, double>> points;
            for (const auto &coord : coordinates) {
                points.emplace_back(coord[0], coord[1]);
            }
            gen_border(points);
        }

        void gen_border(const std::vector<std::pair<double, double>> &coordinates) {
            if (coordinates.size() < 3) {
                throw std::invalid_argument("A polygon must have at least 3 points.");
            }
            border_.outer().clear();
            for (const auto &coord : coordinates) {
                border_.outer().emplace_back(coord.first, coord.second);
            }
            // Ensure the polygon is closed
            if (!bg::equals(border_.outer().front(), border_.outer().back())) {
                border_.outer().emplace_back(border_.outer().front());
            }
            // Correct the polygon's orientation and closure
            bg::correct(border_);
        }

        void gen_field(double swath_width, double angle_degrees, int number = 0) {
            if (number != 0) {
                headlands_ = generate_headlands(swath_width, border_, number);
                auto last_headland = headlands_.back();
                Polygon field_pts_;
                for (const auto &point : last_headland.polygon.outer()) {
                    field_pts_.outer().emplace_back(point.x(), point.y());
                }
                swaths_ = generate_swaths(swath_width, angle_degrees, field_pts_);
            } else {
                swaths_ = generate_swaths(swath_width, angle_degrees, border_);
            }
        }

        void gen_field(const std::vector<Swath> &swaths, double swath_width, int number) {
            headlands_ = generate_headlands(swath_width, border_, number);
            auto last_headland = headlands_.back();

            Polygon field_pts_;
            for (const auto &point : last_headland.polygon.outer()) {
                field_pts_.outer().emplace_back(point.x(), point.y());
            }
            swaths_ = generate_swaths(swaths, field_pts_);
        }

        // Get the swaths as a vector of Swath structs
        const std::vector<Swath> &get_swaths() const { return swaths_; }
        const std::vector<Ring> &get_headlands() const { return headlands_; }
        const Polygon &get_border() const { return border_; }

        // rearrange the swath order from last to first
        void reverse_swaths() { std::reverse(swaths_.begin(), swaths_.end()); }

      private:
        std::vector<Ring> generate_headlands(double x, Polygon polygon_, int number = 1) const {
            if (x < 0) {
                throw std::invalid_argument("Shrink distance must be non-negative.");
            }
            std::vector<Ring> headland_array;
            for (int i = 0; i < number; i++) {
                auto polygon = i == 0 ? polygon_ : headland_array.back().polygon;
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
                headland_array.push_back(
                    {simplifiedPolygon, boost::uuids::to_string(boost::uuids::random_generator()())});
            }
            return headland_array;
        }

        std::vector<Swath> generate_swaths(const std::vector<Swath> &swaths, Polygon field) {
            std::vector<Swath> swaths_;
            for (const auto &swathy : swaths) {
                LineString line = swathy.line;

                std::vector<LineString> clipped;
                boost::geometry::intersection(line, field, clipped);

                for (const auto &segment : clipped) {
                    auto swath_width = boost::geometry::length(segment);
                    if (boost::geometry::length(segment) < swath_width) {
                        continue; // Ignore very short swaths
                    }
                    Swath swath;
                    swath.line = segment;
                    swath.uuid = swathy.uuid;
                    swath.type = SwathType::LINE;
                    swath.length = boost::geometry::length(segment);

                    swaths_.push_back(swath);

                    insert_point_at_closest_location(field, segment.front());
                    insert_point_at_closest_location(field, segment.back());
                }
            }
            return swaths_;
        }

        // Helper function to generate swaths with a specified angle
        std::vector<Swath> generate_swaths(double swath_width, double angle_degrees, Polygon border) {
            std::vector<Swath> swaths;

            // Get the **rotated bounding box**
            Polygon rotatedBoundingBox;
            boost::geometry::convex_hull(border, rotatedBoundingBox);
            boost::geometry::correct(rotatedBoundingBox); // Ensure a valid polygon

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
            auto new_polygon = border;
            for (double offset = -max_dim; offset <= max_dim; offset += swath_width) {
                double length = max_dim * 2; // Extend to ensure full coverage
                LineString swathLine = generate_swathine(centerPoint, angle_radians, offset, length);

                // Clip the swath line to fit within the field polygon
                std::vector<LineString> clipped;
                boost::geometry::intersection(swathLine, border, clipped);

                // Keep all valid segments of the swath that intersect the field polygon
                for (const auto &segment : clipped) {
                    if (boost::geometry::length(segment) < swath_width) {
                        continue; // Ignore very short swaths
                    }
                    Swath swath;
                    swath.line = segment;
                    swath.uuid = boost::uuids::to_string(boost::uuids::random_generator()());
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

        LineString generate_swathine(const Point &first, const Point &second) const {
            LineString swathLine;
            Point newStart(first.x(), first.y());
            Point newEnd(second.x(), second.y());

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

        Polygon get_rotated_bounding_box(const Polygon &polygon) {
            Polygon hullPolygon;
            boost::geometry::convex_hull(polygon, hullPolygon);
            boost::geometry::correct(hullPolygon); // Ensure a valid polygon
            return hullPolygon;
        }
    };

} // namespace farmtrax

#endif // LINE_HPP
