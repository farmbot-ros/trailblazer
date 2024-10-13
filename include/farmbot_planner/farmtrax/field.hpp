#ifndef FIELD_HPP
#define FIELD_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace farmtrax {

    // Define Cartesian point type
    typedef boost::geometry::model::d2::point_xy<double> Point;
    // Define polygon type using the Cartesian point
    typedef boost::geometry::model::polygon<Point> Polygon;
    //define a linestring
    typedef boost::geometry::model::linestring<Point> Linestring;
    // Define a multi-polygon type
    typedef boost::geometry::model::multi_polygon<Polygon> Multipolygon;

    class Field {
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
            if (!boost::geometry::equals(polygon_.outer().front(), polygon_.outer().back())) {
                polygon_.outer().emplace_back(polygon_.outer().front());
            }

            // Optional: Make sure the polygon is correctly oriented (counter-clockwise)
            boost::geometry::correct(polygon_);
        }

        // Get the boundary of the field as a vector of (x, y) coordinates
        std::vector<std::vector<double>> getBoundary() const {
            std::vector<std::vector<double>> boundary;
            for (const auto& point : polygon_.outer()) {
                boundary.push_back({point.x(), point.y()});
            }
            return boundary;
        }

        // Get the polygon representing the field
        const Polygon& getPolygon() const {
            return polygon_;
        }

        // Calculate the area of the field
        double getArea() const {
            return boost::geometry::area(polygon_);
        }

        // Calculate the perimeter of the field
        double getPerimeter() const {
            return boost::geometry::perimeter(polygon_);
        }

        // Calculate the longest side of the field polygon
        double getLongestSide() const {
            double max_length = 0.0;
            const auto& points = polygon_.outer();
            if (points.size() < 2) {
                return max_length;
            }

            for (std::size_t i = 0; i < points.size() - 1; ++i) {
                double dx = points[i+1].x() - points[i].x();
                double dy = points[i+1].y() - points[i].y();
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
            return boost::geometry::within(point, polygon_);
        }

        // Generate a new Field that is "x" meters smaller from every border
    Field getShrunkField(double x) const {
        if (x < 0) {
            throw std::invalid_argument("Shrink distance must be non-negative.");
        }

        // Define buffer strategies with the correct end strategy
        // Negative distance for shrinking
        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(-x);
        boost::geometry::strategy::buffer::side_straight side_strategy;
        boost::geometry::strategy::buffer::join_round join_strategy;
        boost::geometry::strategy::buffer::end_round end_strategy;
        boost::geometry::strategy::buffer::point_circle point_strategy(8); // 8 points per circle for smoothness

        // Perform buffering with negative distance to shrink the polygon
        Multipolygon result;
        boost::geometry::buffer(
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
            double area = boost::geometry::area(poly);
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

    private:
        Polygon polygon_;
        // Field inner_field_;
    };

} // namespace farmtrax

#endif // FIELD_HPP
