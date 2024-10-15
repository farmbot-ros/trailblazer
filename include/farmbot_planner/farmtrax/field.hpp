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

    class Field {
        private:
            Polygon polygon_;
            Rtree rtree_; // R-tree for efficient spatial querying of polygon edges
            std::vector<LineString> edges_; // Store the edges for precise intersection

        public:
            // Constructors
            Field() = default;

            // Initialize with a list of (x, y) coordinates
            Field(const std::vector<std::pair<double, double>>& coordinates) {
                setBoundary(coordinates);
            }

            // Set the boundary of the field using a list of (x, y) coordinates
            void setBoundary(const std::vector<std::pair<double, double>>& coordinates) {
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
                insertPolygonEdgesIntoRtree();
            }

            // Get the boundary of the field as a vector of (x, y) coordinates
            std::vector<std::pair<double, double>> getBoundary() const {
                std::vector<std::pair<double, double>> boundary;
                for (const auto& point : polygon_.outer()) {
                    boundary.emplace_back(point.x(), point.y());
                }
                return boundary;
            }

            // Get the polygon representing the field
            const Polygon& getPolygon() const {
                return polygon_;
            }

            // Calculate the area of the field
            double getArea() const {
                return bg::area(polygon_);
            }

            // Calculate the perimeter of the field
            double getPerimeter() const {
                return bg::perimeter(polygon_);
            }

            // Calculate the longest side of the field polygon
            double getLongestSide() const {
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
                        // Insert polygon edges into the R-tree
            void insertPolygonEdgesIntoRtree() {
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

            Field getBuffered(double distance, BufferType type) const {
                switch (type) {
                    case BufferType::SHRINK:
                        return getShrunkField(distance);
                    case BufferType::ENLARGE:
                        return getEnlargedField(distance);
                    default:
                        throw std::invalid_argument("Invalid buffer type.");
                }
            }
        private:
            // Generate a new Field that is "x" meters smaller from every border
            Field getShrunkField(double x) const {
                if (x < 0) {
                    throw std::invalid_argument("Shrink distance must be non-negative.");
                }
                // Define buffer strategies with the correct end strategy
                // Negative distance for shrinking
                bg::strategy::buffer::distance_symmetric<double> distance_strategy(-x);
                bg::strategy::buffer::side_straight side_strategy;
                bg::strategy::buffer::join_round join_strategy;
                bg::strategy::buffer::end_round end_strategy;
                bg::strategy::buffer::point_circle point_strategy(8); // 8 points per circle for smoothness
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
                // Convert the largest polygon back to a Field
                Field shrunkField;
                std::vector<std::pair<double, double>> shrunkBoundary;
                for (const auto& point : largest->outer()) {
                    shrunkBoundary.emplace_back(point.x(), point.y());
                }
                shrunkField.setBoundary(shrunkBoundary);
                return shrunkField;
            }

            // Generate a new Field that is "x" meters larger from every border
            Field getEnlargedField(double x) const {
                if (x < 0) {
                    throw std::invalid_argument("Enlargement distance must be non-negative.");
                }
                // Define buffer strategies with the correct end strategy
                // Positive distance for enlarging
                bg::strategy::buffer::distance_symmetric<double> distance_strategy(x);
                bg::strategy::buffer::side_straight side_strategy;
                bg::strategy::buffer::join_round join_strategy;
                bg::strategy::buffer::end_round end_strategy;
                bg::strategy::buffer::point_circle point_strategy(8); // 8 points per circle for smoothness

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

                // Convert the largest polygon back to a Field
                Field enlargedField;
                std::vector<std::pair<double, double>> enlargedBoundary;
                for (const auto& point : largest->outer()) {
                    enlargedBoundary.emplace_back(point.x(), point.y());
                }

                enlargedField.setBoundary(enlargedBoundary);
                return enlargedField;
            }
    };

} // namespace farmtrax

#endif // FIELD_HPP
