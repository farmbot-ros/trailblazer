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
        std::vector<std::pair<double, double>> getBoundary() const {
            std::vector<std::pair<double, double>> boundary;
            for (const auto& point : polygon_.outer()) {
                boundary.emplace_back(point.x(), point.y());
            }
            return boundary;
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

    private:
        Polygon polygon_;
    };

} // namespace farmtrax

#endif // FIELD_HPP
