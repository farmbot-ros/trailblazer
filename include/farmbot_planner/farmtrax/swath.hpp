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
        private:
            Field inner_field_;
            Field outer_field_;
            double swath_width_;
            std::vector<Linestring> swaths_;

        public:
            Swath() = default;

            // Constructor to initialize with a field and swath width
            Swath(const Field& outer_field, const Field& inner_field, double swath_width)
                : outer_field_(outer_field), inner_field_(inner_field), swath_width_(swath_width) {
                generateSwaths();
            }

            // Set the field and regenerate swaths
            void setFields(const Field& outer_field, const Field& inner_field) {
                outer_field_ = outer_field;
                inner_field_ = inner_field;
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
            // Helper function to generate swaths with a specified angle
            void generateSwaths(double angle_degrees=45) {
                swaths_.clear();
                Field new_field = inner_field_.getEnlargedField(0.5);
                Polygon fieldPolygon = new_field.getPolygon();

                // Get the bounding box of the field
                boost::geometry::model::box<Point> boundingBox;
                boost::geometry::envelope(fieldPolygon, boundingBox);

                // Convert angle from degrees to radians
                double angle_radians = angle_degrees * M_PI / 180.0;

                // Determine the dimensions of the bounding box
                double width = boundingBox.max_corner().x() - boundingBox.min_corner().x();
                double height = boundingBox.max_corner().y() - boundingBox.min_corner().y();
                double max_dim = std::max(width, height);

                // Starting point and ending point adjusted for angle
                Point centerPoint((boundingBox.min_corner().x() + boundingBox.max_corner().x()) / 2,
                                (boundingBox.min_corner().y() + boundingBox.max_corner().y()) / 2);

                // Iterate to generate swaths with a defined offset based on swath width
                for (double offset = -max_dim / 2; offset <= max_dim / 2; offset += swath_width_) {
                    Linestring swathLine = generateSwathLine(centerPoint, angle_radians, offset);

                    // Clip the swath line to fit within the field polygon
                    std::vector<Linestring> clipped;
                    boost::geometry::intersection(swathLine, fieldPolygon, clipped);

                    // Keep all valid segments of the swath that intersect the field polygon
                    for (const auto& segment : clipped) {
                        swaths_.push_back(segment);
                    }
                }
            }

            // Function to generate a line at a certain offset from the center, adjusted for the angle
            Linestring generateSwathLine(const Point& center, double angle_radians, double offset) const {
                Linestring swathLine;

                // Calculate the perpendicular offset direction based on the angle
                double cos_angle = std::cos(angle_radians);
                double sin_angle = std::sin(angle_radians);

                // Define the half-length of the swath lines (to extend equally from the center)
                double half_length = 1000.0; // A large value to ensure swath lines extend beyond the field boundary

                // Calculate the start and end points of the swath line
                Point newStart(center.x() + (offset * sin_angle) - (half_length * cos_angle),
                            center.y() - (offset * cos_angle) - (half_length * sin_angle));

                Point newEnd(center.x() + (offset * sin_angle) + (half_length * cos_angle),
                            center.y() - (offset * cos_angle) + (half_length * sin_angle));

                // Add the new start and end points to the swath line
                swathLine.push_back(newStart);
                swathLine.push_back(newEnd);

                return swathLine;
            }


    };

} // namespace farmtrax

#endif // SWATH_HPP
