#ifndef FIELD_HPP
#define FIELD_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <limits> // For numeric_limits

namespace farmtrax {

    enum class BufferType {
        SHRINK,
        ENLARGE
    };

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

    // Define the R-tree value type: pair of Box and an identifier (size_t)
    typedef std::pair<Box, std::size_t> RtreeValue;
    typedef bgi::rtree<RtreeValue, bgi::quadratic<16>> Rtree;

    // Function to check if three points are colinear
    template<typename Point>
    bool are_colinear(const Point& p1, const Point& p2, const Point& p3, double epsilon = 1e-10) {
        // Calculate the area of the triangle formed by the three points
        // If the area is zero (or close to zero), the points are colinear

        auto area = (boost::geometry::get<0>(p1) * (boost::geometry::get<1>(p2) - boost::geometry::get<1>(p3)) +
                    boost::geometry::get<0>(p2) * (boost::geometry::get<1>(p3) - boost::geometry::get<1>(p1)) +
                    boost::geometry::get<0>(p3) * (boost::geometry::get<1>(p1) - boost::geometry::get<1>(p2))) / 2.0;

        return std::abs(area) < epsilon;
    }

    // Function to remove colinear points from a polygon
    template<typename Polygon>
    void remove_colinear_points(Polygon& polygon, double epsilon = 1e-10) {
        using point_type = typename boost::geometry::point_type<Polygon>::type;
        std::vector<point_type> new_points;
        auto const& points = polygon.outer();

        if (points.size() < 4) return; // Need at least 3 distinct points (excluding duplicate endpoint)

        // Since the polygon is closed, the first and last points are the same.
        // We iterate excluding the duplicate last point.
        size_t n = points.size() - 1;
        for (size_t i = 0; i < n; ++i) {
            const point_type& prev = points[(i + n - 1) % n];
            const point_type& curr = points[i];
            const point_type& next = points[(i + 1) % n];

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


    class Field {
        private:
            Polygon polygon_;
            Rtree rtree_; // R-tree for efficient spatial querying of polygon edges
            std::vector<LineString> edges_; // Store the edges for precise intersection
            double width_;
            double height_;

            double colinear_threshold_ = 0.01;

        public:
            // Constructors
            Field() = default;

            // Initialize with a list of (x, y) coordinates
            Field(const std::vector<std::pair<double, double>>& coordinates) {
                gen_field(coordinates);
            }

            // Set the boundary of the field using a list of (x, y) coordinates
            void gen_field(const std::vector<std::pair<double, double>>& coordinates) {
                if (coordinates.size() < 3) {
                    throw std::invalid_argument("A polygon must have at least 3 points.");
                }
                polygon_.outer().clear();
                for (const auto& coord : coordinates) {
                    polygon_.outer().emplace_back(coord.first, coord.second);
                }
                // Ensure the polygon is closed
                if (!bg::equals(polygon_.outer().front(), polygon_.outer().back())) {
                    polygon_.outer().emplace_back(polygon_.outer().front());
                }
                // Correct the polygon's orientation and closure
                bg::correct(polygon_);
                // After setting the boundary, insert the polygon's edges into the R-tree
                const auto& outer_ring = polygon_.outer();
                for (std::size_t i = 0; i < outer_ring.size() - 1; ++i) {
                    LineString edge;
                    bg::append(edge, outer_ring[i]);
                    bg::append(edge, outer_ring[i + 1]);
                    // Compute the envelope of the edge
                    Box box;
                    bg::envelope(edge, box);
                    // Insert the box and index into the R-tree
                    rtree_.insert(std::make_pair(box, i));
                    // Store the edge for precise intersection checks
                    edges_.push_back(edge);
                }
                // Compute the width and height of the field
                Box bbox;
                bg::envelope(polygon_, bbox);
                width_ = bbox.max_corner().x() - bbox.min_corner().x();
                height_ = bbox.max_corner().y() - bbox.min_corner().y();
            }

            // Get the boundary of the field as a vector of (x, y) coordinates
            std::vector<std::pair<double, double>> get_border_points() const {
                std::vector<std::pair<double, double>> boundary;
                for (const auto& point : polygon_.outer()) {
                    boundary.emplace_back(point.x(), point.y());
                }
                return boundary;
            }

            // Get the polygon representing the field
            const Polygon& get_polygon() const {
                return polygon_;
            }

            //get edges of the field
            const std::vector<LineString>& get_edges() const {
                return edges_;
            }

            // Calculate the area of the field
            double get_area() const {
                return bg::area(polygon_);
            }

            // Calculate the perimeter of the field
            double get_perimeter() const {
                return bg::perimeter(polygon_);
            }

            // Calculate the longest side of the field polygon
            double get_longest_side() const {
                double max_length = 0.0;
                const auto& points = polygon_.outer();
                if (points.size() < 2) {
                    return max_length;
                }

                for (std::size_t i = 0; i < points.size() - 1; ++i) {
                    double dx = points[i + 1].x() - points[i].x();
                    double dy = points[i + 1].y() - points[i].y();
                    double length = std::sqrt(dx * dx + dy * dy);
                    if (length > max_length) {
                        max_length = length;
                    }
                }

                return max_length;
            }

            // Check if a point is inside the field
            bool contains(double x, double y) const {
                Point point(x, y);
                return bg::within(point, polygon_);
            }

            // Check if a linestring is inside the field
            bool contains(const LineString& line) const {
                return bg::within(line, polygon_);
            }

            // Check if a LineString intersects the polygon using the R-tree
            bool intersects(const LineString& line) const {
                // Compute the envelope of the connection
                Box connection_box;
                bg::envelope(line, connection_box);
                // Query the R-tree for candidate edges whose boxes intersect the connection's box
                std::vector<RtreeValue> results;
                rtree_.query(bgi::intersects(connection_box), std::back_inserter(results));
                // For each candidate edge, check if it intersects with the connection LineString
                for (const auto& value : results) {
                    const LineString& edge = edges_[value.second];
                    if (bg::intersects(line, edge)) {
                        return true;
                    }
                }
                return false;
            }

            Field get_buffered(double distance, BufferType type) const {
                switch (type) {
                    case BufferType::SHRINK:
                        return get_shrunk_field(distance);
                    case BufferType::ENLARGE:
                        return get_enlarged_field(distance);
                    default:
                        throw std::invalid_argument("Invalid buffer type.");
                }
            }

            // Get the width of the field (x-axis)
            double get_width() const {
                return width_;
            }
            
            // Get the height of the field (y-axis)
            double get_height() const {
                return height_;
            }

        private:
            // Generate a new Field that is "x" meters smaller from every border
            Field get_shrunk_field(double x) const {
                if (x < 0) {
                    throw std::invalid_argument("Shrink distance must be non-negative.");
                }

                // Define buffer strategies with straight edges
                bg::strategy::buffer::distance_symmetric<double> distance_strategy(-x);
                bg::strategy::buffer::side_straight side_strategy;
                bg::strategy::buffer::join_miter join_strategy;
                bg::strategy::buffer::end_flat end_strategy;
                bg::strategy::buffer::point_square point_strategy;

                // Perform buffering with negative distance to shrink the polygon
                Multipolygon result;
                bg::buffer(
                    polygon_,
                    result,
                    distance_strategy,
                    side_strategy,
                    join_strategy,
                    end_strategy,
                    point_strategy
                );

                if (result.empty()) {
                    throw std::runtime_error("Shrinking resulted in an empty field.");
                }

                // Select the largest polygon from the result
                const Polygon* largest = nullptr;
                double max_area = -std::numeric_limits<double>::max();
                for (const auto& poly : result) {
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

                // Convert the largest polygon back to a Field
                Field shrunkField;
                std::vector<std::pair<double, double>> shrunkBoundary;
                for (const auto& point : simplifiedPolygon.outer()) {
                    shrunkBoundary.emplace_back(point.x(), point.y());
                }
                shrunkField.gen_field(shrunkBoundary);
                return shrunkField;
            }

            // Generate a new Field that is "x" meters larger from every border
            Field get_enlarged_field(double x) const {
                if (x < 0) {
                    throw std::invalid_argument("Enlargement distance must be non-negative.");
                }
                // Define buffer strategies with the correct end strategy
                // Positive distance for enlarging
                bg::strategy::buffer::distance_symmetric<double> distance_strategy(x);
                bg::strategy::buffer::side_straight side_strategy;
                bg::strategy::buffer::join_round join_strategy;
                bg::strategy::buffer::end_round end_strategy;
                bg::strategy::buffer::point_square point_strategy;

                // Perform buffering with positive distance to enlarge the polygon
                Multipolygon result;
                bg::buffer(
                    polygon_,
                    result,
                    distance_strategy,
                    side_strategy,
                    join_strategy,
                    end_strategy,
                    point_strategy
                );
                if (result.empty()) {
                    throw std::runtime_error("Enlarging resulted in an empty field.");
                }

                // Select the largest polygon from the result
                const Polygon* largest = nullptr;
                double max_area = -std::numeric_limits<double>::max();
                for (const auto& poly : result) {
                    double area = bg::area(poly);
                    if (area > max_area) {
                        max_area = area;
                        largest = &poly;
                    }
                }
                if (!largest) {
                    throw std::runtime_error("Failed to determine the largest polygon after enlarging.");
                }

                Polygon simplifiedPolygon = *largest;
                remove_colinear_points(simplifiedPolygon, colinear_threshold_);

                // Convert the largest polygon back to a Field
                Field enlargedField;
                std::vector<std::pair<double, double>> enlargedBoundary;
                for (const auto& point : simplifiedPolygon.outer()) {
                    enlargedBoundary.emplace_back(point.x(), point.y());
                }

                enlargedField.gen_field(enlargedBoundary);
                return enlargedField;
            }
    };

} // namespace farmtrax

#endif // FIELD_HPP
