#ifndef SWATH_HPP
#define SWATH_HPP

#include "field.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <boost/geometry/algorithms/distance.hpp> // For calculating distance

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.

#include <vector>
#include <string>
#include <cmath>

namespace farmtrax {

    // Enum class to represent different types of swaths
    enum class SwathType {
        LAND,
        HEAD,
        PATH
    };

    // Struct to represent each swath, along with its properties
    struct Swath {
        Linestring swath;         // The actual swath line (geometry)
        std::string uuid;         // A unique identifier for each swath
        SwathType type;           // The type of swath (LAND, HEAD, PATH)
        bool transportlane;       // Flag to indicate if it's a transport lane
    };

    class Swaths {
    private:
        Field inner_field_;
        Field outer_field_;
        double swath_width_;
        std::vector<Swath> swaths_;  // Holds Swath structs

    public:
        Swaths() = default;

        // Constructor to initialize with a field and swath width
        Swaths(const Field& outer_field, const Field& inner_field, double swath_width)
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

        // Get the swaths as a vector of Swath structs
        std::vector<Swath> getSwaths() const {
            return swaths_;
        }

void connectSwathsInUShape() {
    // Ensure all swaths are oriented in the same direction
    for (size_t i = 0; i < swaths_.size(); ++i) {
        // For every other swath, reverse the swath so they all point in the same direction
        if (i % 2 == 1) {
            std::reverse(swaths_[i].swath.begin(), swaths_[i].swath.end());
        }
    }

    // Vector to hold the connecting lines between swath heads and tails
    std::vector<Linestring> connecting_lines;

    // Now connect swaths in a U-shaped manner (end of one swath to the start of the next)
    for (size_t i = 0; i < swaths_.size() - 1; ++i) {
        // Always connect the end of the current swath to the start of the next swath
        const Point& current_end = swaths_[i].swath.back();     // End of the current swath
        const Point& next_start = swaths_[i + 1].swath.front(); // Start of the next swath

        // Create a connecting line between current swath and next swath
        Linestring connecting_line;
        connecting_line.push_back(current_end);
        connecting_line.push_back(next_start);

        // Add the connecting line to the vector
        connecting_lines.push_back(connecting_line);
    }

    // Append the connecting lines to swaths_ as new Swaths
    for (const auto& line : connecting_lines) {
        Swath connecting_swath;
        connecting_swath.swath = line;
        connecting_swath.uuid = generateUUID();  // Generate UUID for the connecting line
        connecting_swath.type = SwathType::HEAD; // Set as HEAD type for U-shaped connection
        connecting_swath.transportlane = false;  // Default, can modify later if needed

        // Add the new connecting swath to the swaths_ vector
        swaths_.push_back(connecting_swath);
    }

    // No connection is made between the last swath and the first
}





    private:
        // Helper function to generate swaths with a specified angle
        void generateSwaths(double angle_degrees = 45) {
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
                    Swath swath;  // Create a Swath struct for each segment
                    swath.swath = segment;
                    swath.uuid = generateUUID();  // Generate a unique ID for each swath
                    swath.type = SwathType::LAND; // Default type, can modify as needed
                    swath.transportlane = false;  // Default, can modify as needed

                    swaths_.push_back(swath);
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

        // Function to generate a unique identifier for each swath
        std::string generateUUID() const {
            return boost::uuids::to_string(boost::uuids::random_generator()());
        }
    };

} // namespace farmtrax

#endif // SWATH_HPP
