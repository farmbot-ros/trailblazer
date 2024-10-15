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
#include <map>
#include <queue>

namespace farmtrax {
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
    enum class SwathType {
        LAND,
        HEAD,
        PATH
    };

    // Struct to represent each swath, along with its properties
    struct Swath {
        LineString swath;         // The actual swath line (geometry)
        std::string uuid;         // A unique identifier for each swath
        SwathType type;           // The type of swath (LAND, HEAD, PATH)
        bool transportlane;       // Flag to indicate if it's a transport lane
    };

    class Swaths {
    private:
        Field inner_field_;
        Field outer_field_;
        double swath_width_;
        double angle_degrees_;
        std::vector<Swath> swaths_;  // Holds Swath structs

    public:
        Swaths() = default;

        // Constructor to initialize with a field and swath width
        Swaths(const Field& outer_field, const Field& inner_field, double swath_width, double angle_degrees) {
            genSwaths(outer_field, inner_field, swath_width, angle_degrees);
        }

        void genSwaths(const Field& outer_field, const Field& inner_field, double swath_width, double angle_degrees) {
            outer_field_ = outer_field;
            inner_field_ = inner_field;
            swath_width_ = swath_width;
            angle_degrees_ = angle_degrees;
            generateSwaths();
        }

        // Get the swaths as a vector of Swath structs
        std::vector<Swath> getSwaths() const {
            return swaths_;
        }

        // Function to connect swaths in a U-shape
        void connectSwathsInUShape() {
            // Ensure all swaths are oriented in the same direction
            for (size_t i = 0; i < swaths_.size(); ++i) {
                // For every other swath, reverse the swath so they all point in the same direction
                if (i % 2 == 1) {
                    std::reverse(swaths_[i].swath.begin(), swaths_[i].swath.end());
                }
            }

            // Vector to hold the connecting lines between swath heads and tails
            std::vector<LineString> connecting_lines;

            // Extract endpoints of swaths
            std::vector<std::pair<Point, bool>> endpoints; // Point, is_start
            for (const auto& swath : swaths_) {
                endpoints.emplace_back(swath.swath.front(), true);  // Start point
                endpoints.emplace_back(swath.swath.back(), false);  // End point
            }

            // Connect swaths
            for (size_t i = 0; i < swaths_.size() - 1; ++i) {
                const Point& current_end = swaths_[i].swath.back();
                const Point& next_start = swaths_[i + 1].swath.front();

                if (!intersectsPolygon(current_end, next_start)) {
                    LineString connection = createConnection(current_end, next_start);
                    addConnectingSwath(connection);  // Add as HEAD
                }
            }
        }

    private:
        // Helper function to generate swaths with a specified angle
        void generateSwaths() {
            swaths_.clear();
            Field new_field = inner_field_.getBuffered(0.5, farmtrax::BufferType::ENLARGE);
            Polygon fieldPolygon = new_field.getPolygon();

            // Get the bounding box of the field
            boost::geometry::model::box<Point> boundingBox;
            boost::geometry::envelope(fieldPolygon, boundingBox);

            // Convert angle from degrees to radians
            double angle_radians = angle_degrees_ * M_PI / 180.0;

            // Determine the dimensions of the bounding box
            double width = boundingBox.max_corner().x() - boundingBox.min_corner().x();
            double height = boundingBox.max_corner().y() - boundingBox.min_corner().y();
            double max_dim = std::max(width, height);

            // Starting point and ending point adjusted for angle
            Point centerPoint((boundingBox.min_corner().x() + boundingBox.max_corner().x()) / 2,
                            (boundingBox.min_corner().y() + boundingBox.max_corner().y()) / 2);

            // Iterate to generate swaths with a defined offset based on swath width
            for (double offset = -max_dim / 2; offset <= max_dim / 2; offset += swath_width_) {
                LineString swathLine = generateSwathLine(centerPoint, angle_radians, offset);

                // Clip the swath line to fit within the field polygon
                std::vector<LineString> clipped;
                boost::geometry::intersection(swathLine, fieldPolygon, clipped);

                // Keep all valid segments of the swath that intersect the field polygon
                for (const auto& segment : clipped) {
                    Swath swath;  // Create a Swath struct for each segment
                    swath.swath = segment;
                    swath.uuid = generateUUID();  // Generate a unique ID for each swath
                    swath.type = SwathType::LAND; // Always mark as LAND here
                    swath.transportlane = false;  // Default, can modify as needed

                    swaths_.push_back(swath);
                }
            }
        }

        // Function to generate a line at a certain offset from the center, adjusted for the angle
        LineString generateSwathLine(const Point& center, double angle_radians, double offset) const {
            LineString swathLine;

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

        // Function to check if a connection intersects the polygon
        bool intersectsPolygon(const Point& p1, const Point& p2) const {
            LineString connection;
            connection.push_back(p1);
            connection.push_back(p2);

            return boost::geometry::intersects(connection, inner_field_.getPolygon());
        }

        // Function to create a connection between two points
        LineString createConnection(const Point& p1, const Point& p2) const {
            LineString connection;
            connection.push_back(p1);
            connection.push_back(p2);
            return connection;
        }

        // Function to add a connecting swath to the swath list
        void addConnectingSwath(const LineString& connection) {
            Swath connecting_swath;
            connecting_swath.swath = connection;
            connecting_swath.uuid = generateUUID();
            connecting_swath.type = SwathType::HEAD;  // Only connections are labeled as HEAD
            connecting_swath.transportlane = false;   // Can be modified as needed
            swaths_.push_back(connecting_swath);
        }

        // Function to generate a unique identifier for each swath
        std::string generateUUID() const {
            return boost::uuids::to_string(boost::uuids::random_generator()());
        }
    };

} // namespace farmtrax

#endif // SWATH_HPP
