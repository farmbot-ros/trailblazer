#ifndef SWATH_HPP
#define SWATH_HPP

#include "field.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <vector>

namespace farmtrax {

    class Swath {
    public:
        Swath() = default;

        // Constructor to initialize with a field and swath width
        Swath(const Field& field, double swath_width)
            : field_(field), swath_width_(swath_width) {
            generateSwaths();
        }

        // Set the field and regenerate swaths
        void setField(const Field& field) {
            field_ = field;
            generateSwaths();
        }

        // Set the swath width and regenerate swaths
        void setSwathWidth(double swath_width) {
            swath_width_ = swath_width;
            generateSwaths();
        }

        // Get the swaths as a vector of linestrings (each line represents a swath)
        std::vector<Linestring> getSwaths() const {
            return swaths_;
        }

        std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> getSwathsAsVector() const {
            std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> swaths;
            for (const auto& swath : swaths_) {
                swaths.push_back({{swath.front().x(), swath.front().y()}, {swath.back().x(), swath.back().y()}});
            }
            return swaths;
        }

    private:
        Field field_;
        double swath_width_;
        std::vector<Linestring> swaths_;

        // Helper function to generate swaths
        void generateSwaths() {
            swaths_.clear();
            Polygon fieldPolygon = field_.getPolygon(); // Assume you add a getPolygon() method in Field

            // Get the bounding box of the field (a square that fits the entire field)
            boost::geometry::model::box<Point> boundingBox;
            boost::geometry::envelope(fieldPolygon, boundingBox);

            // Expand the bounding box into a square by taking the larger dimension
            double width = boundingBox.max_corner().x() - boundingBox.min_corner().x();
            double height = boundingBox.max_corner().y() - boundingBox.min_corner().y();
            double max_dim = std::max(width, height);

            // Create a square by expanding the bounding box
            boost::geometry::model::box<Point> squareBox(
                boundingBox.min_corner(),
                Point(boundingBox.min_corner().x() + max_dim, boundingBox.min_corner().y() + max_dim)
            );

            // Generate swath lines within the square
            Point startPoint(squareBox.min_corner().x(), squareBox.min_corner().y());
            Point endPoint(squareBox.min_corner().x(), squareBox.max_corner().y());

            for (double offset = 0; offset <= max_dim; offset += swath_width_) {
                Linestring swathLine = generateSwathLine(startPoint, endPoint, offset);

                // Clip the swath line to fit within the field polygon
                std::vector<Linestring> clipped;
                boost::geometry::intersection(swathLine, fieldPolygon, clipped);

                // Keep all valid segments of the swath that intersect the field polygon
                for (const auto& segment : clipped) {
                    if (!segment.empty()) {
                        swaths_.push_back(segment);
                    }
                }
            }
        }

        // Function to generate a line at a certain offset from the original start and end points
        Linestring generateSwathLine(const Point& start, const Point& end, double offset) const {
            Linestring swathLine;
            double dx = end.x() - start.x();
            double dy = end.y() - start.y();
            double length = std::sqrt(dx * dx + dy * dy);

            // Normalize the direction vector (dx, dy)
            double nx = dx / length;
            double ny = dy / length;

            // Apply the offset in the perpendicular direction
            Point newStart(start.x() + offset * ny, start.y() - offset * nx);
            Point newEnd(end.x() + offset * ny, end.y() - offset * nx);

            swathLine.push_back(newStart);
            swathLine.push_back(newEnd);

            return swathLine;
        }
    };

} // namespace farmtrax

#endif // SWATH_HPP
