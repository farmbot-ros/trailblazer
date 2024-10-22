#ifndef SWATH_HPP
#define SWATH_HPP

#include "field.hpp"
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.

#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <queue>
#include <utility> // For std::pair

namespace farmtrax {
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

    // Enum class to represent different types of swaths
    enum class SwathType {
        LINE,
        TURN,
        ROAD
    };

    enum class Direction {
        FORWARD,
        REVERSE,
        ANY
    };

    // Struct to represent each swath, along with its properties
    struct Swath {
        LineString swath;         // The actual swath line (geometry)
        std::string uuid;         // A unique identifier for each swath
        SwathType type;           // The type of swath (LINE, TURN, PATH)
        Direction direction;      // The direction of the swath (FORWARD, REVERSE)
        double length;             // Length of the swath

        bool intersects(const Field& field) const {
            Polygon fieldPolygon = field.get_polygon();
            return bg::intersects(fieldPolygon, swath);
        }

        Swath reverse() const {
            Swath new_swath = *this;
            LineString reversed_swath = swath;
            std::reverse(reversed_swath.begin(), reversed_swath.end());
            auto new_uuid = boost::uuids::to_string(boost::uuids::random_generator()());
            auto new_direction = (this->direction == Direction::FORWARD) ? Direction::REVERSE : Direction::FORWARD;
            new_swath.swath = reversed_swath;
            new_swath.uuid = new_uuid;
            new_swath.direction = new_direction;
            return new_swath;
        }

        Swath create_swath(const Point& start, const Point& end, SwathType type, Direction direction, std::string uuid = "") {
            LineString line;
            bg::append(line, start);
            bg::append(line, end);
            std::string uuid_ = uuid.empty() ? boost::uuids::to_string(boost::uuids::random_generator()()) : uuid;
            return {line, uuid_, type, direction, 0.0};
        }
    };

    // R-tree type definitions
    typedef std::pair<Box, std::size_t> RtreeValue;
    typedef bgi::rtree<RtreeValue, bgi::quadratic<16>> Rtree;

    class Swaths {
        private:
            std::vector<Swath> swaths_;             // Holds Swath structs
            Rtree swath_rtree_;                     // R-tree for efficient spatial querying of swaths
            std::vector<Swath> with_headland_;      // Holds Swath structs with turns
            Polygon connecting_polygon_;            // Polygon to represent the connecting swath

        public:
            Swaths() = default;

            // Constructor to initialize with a field and swath width
            Swaths(const Field& outer_field, const Field& inner_field, double swath_width, double angle_degrees, int alternate_freq = 1, double inner_offset = 1.0) {
                gen_swaths(outer_field, inner_field, swath_width, angle_degrees, alternate_freq, inner_offset);
            }

            void gen_swaths(const Field& outer_field, const Field& inner_field, double swath_width, double angle_degrees, int alternate_freq = 1, double inner_offset = 1.0) {
                generate_swaths(outer_field, inner_field, swath_width, angle_degrees, alternate_freq, inner_offset);
            }

            // Get the swaths as a vector of Swath structs
            const std::vector<Swath>& get_swaths() const {
                return swaths_;
            }

            // Get the swaths with headlands as a vector of Swath structs
            const std::vector<Swath>& get_swaths_with_headland() const {
                return with_headland_;
            }

            // Add a swath to the list of swaths
            void add_swath(const Swath& swath) {
                swaths_.push_back(swath);
            }

            // If swath intersects with field
            bool intersects_field(const Field& field, const Swath& swath) {
                Polygon fieldPolygon = field.get_polygon();
                LineString swathLine = swath.swath;
                return boost::geometry::intersects(fieldPolygon, swathLine);
            }

            // Query swaths intersecting a given bounding box
            std::vector<std::size_t> query_swaths(const Box& query_box) const {
                std::vector<RtreeValue> result_s;
                swath_rtree_.query(bgi::intersects(query_box), std::back_inserter(result_s));

                std::vector<std::size_t> swath_indices;
                swath_indices.reserve(result_s.size());
                for (const auto& val : result_s) {
                    swath_indices.push_back(val.second);
                }
                return swath_indices;
            }

            // Find the nearest swath to a given point
            std::size_t nearest_swath(const Point& point) const {
                std::vector<RtreeValue> result_s;
                swath_rtree_.query(bgi::nearest(point, 1), std::back_inserter(result_s));

                if (!result_s.empty()) {
                    return result_s.front().second;
                } else {
                    throw std::runtime_error("No swaths available.");
                }
            }

            //check if two points are connected by a swath
            bool are_connected(const Point& p1, const Point& p2) {
                LineString connection = create_connection(p1, p2);
                for (const auto& swath : swaths_) {
                    if (bg::intersects(connection, swath.swath)) {
                        return true;
                    }
                }
                return false;
            }

            // get connecting polygon
            const Polygon get_connecting_polygon() const {
                return connecting_polygon_;
            }

        private:
            // Helper function to generate swaths with a specified angle
            void generate_swaths(const Field& outer_field_, const Field& inner_field_, double swath_width, double angle_degrees, int alternate_freq = 1, double inner_offset = 1.0) {
                swaths_.clear();
                swath_rtree_.clear(); // Clear existing entries


                Field new_field = inner_field_.get_buffered(inner_offset, farmtrax::BufferType::ENLARGE);
                Polygon fieldPolygon = new_field.get_polygon();

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
                bool alternate = true;
                int counter = 0;
                auto new_polygon = fieldPolygon;
                for (double offset = -max_dim / 2; offset <= max_dim / 2; offset += swath_width) {
                    counter++;
                    // Alternate the direction of the swaths based on the frequency
                    alternate = (counter % alternate_freq == 0) ? !alternate : alternate;

                    double length = std::max(outer_field_.get_width(), outer_field_.get_height()) * 2;
                    LineString swathLine = generate_swathine(centerPoint, angle_radians, offset, length);
                    // Clip the swath line to fit within the field polygon

                    std::vector<LineString> clipped;
                    boost::geometry::intersection(swathLine, fieldPolygon, clipped);
                    // Keep all valid segments of the swath that intersect the field polygon

                    for (const auto& segment : clipped) {
                        //if lenght is smaller than swath width, then ignore it
                        if (bg::length(segment) < swath_width) {
                            continue;
                        }
                        Swath swath;  // Create a Swath struct for each segment
                        swath.swath = segment;
                        swath.uuid = generate_UUID();  // Generate a unique ID for each swath
                        swath.type = SwathType::LINE; // Always mark as LINE here
                        swath.length = bg::length(segment); // Calculate the length of the swath
                        swath.direction = alternate ? Direction::FORWARD : Direction::REVERSE;
                        swaths_.push_back(swath);

                        insert_point_at_closest_location(new_polygon, segment.front());
                        insert_point_at_closest_location(new_polygon, segment.back());

                        // Insert the swath into the R-tree
                        Box swath_box;
                        boost::geometry::envelope(segment, swath_box);
                        swath_rtree_.insert(std::make_pair(swath_box, swaths_.size() - 1));
                    }
                }
                connecting_polygon_ = new_polygon;
                with_headland_ = swaths_;
                for (long unsigned int i = 0; i < connecting_polygon_.outer().size(); i++) {
                    Swath swath;
                    swath.swath = create_connection(connecting_polygon_.outer()[i], connecting_polygon_.outer()[(i + 1) % connecting_polygon_.outer().size()]);
                    swath.uuid = generate_UUID();
                    swath.type = SwathType::TURN;
                    swath.length = bg::distance(connecting_polygon_.outer()[i], connecting_polygon_.outer()[(i + 1) % connecting_polygon_.outer().size()]);
                    swath.direction = Direction::ANY;
                    with_headland_.push_back(swath);
                }
            }

            // Function to generate a line at a certain offset from the center, adjusted for the angle
            LineString generate_swathine(const Point& center, double angle_radians, double offset, double length) const {
                LineString swathLine;

                // Calculate the perpendicular offset direction based on the angle
                double cos_angle = std::cos(angle_radians);
                double sin_angle = std::sin(angle_radians);

                // Calculate the start and end points of the swath line
                Point newStart(center.x() + (offset * sin_angle) - (length * cos_angle),
                            center.y() - (offset * cos_angle) - (length * sin_angle));

                Point newEnd(center.x() + (offset * sin_angle) + (length * cos_angle),
                            center.y() - (offset * cos_angle) + (length * sin_angle));

                // Add the new start and end points to the swath line
                swathLine.push_back(newStart);
                swathLine.push_back(newEnd);

                return swathLine;
            }

            void insert_point_at_closest_location(Polygon& poly, const Point& p){
                auto& outer_ring = poly.outer();
                // Check if the ring is closed (first and last points are the same)
                bool is_closed = !outer_ring.empty() && bg::equals(outer_ring.front(), outer_ring.back());
                // Remove the closing point if the ring is closed
                if (is_closed){
                    outer_ring.pop_back();
                }
                // Variables to keep track of the closest segment
                double min_distance = std::numeric_limits<double>::max();
                size_t insert_position = 0; // Position to insert the point
                // Iterate over the segments of the outer ring
                for (size_t i = 0; i < outer_ring.size(); ++i){
                    // Get the current segment
                    Point p1 = outer_ring[i];
                    Point p2 = outer_ring[(i + 1) % outer_ring.size()]; // Wrap around for the last segment
                    bg::model::segment<Point> seg(p1, p2);
                    // Compute the distance from the point to the segment
                    double distance = bg::distance(p, seg);
                    // Update the minimum distance and insertion position if necessary
                    if (distance < min_distance){
                        min_distance = distance;
                        insert_position = i + 1; // Insert after point i
                    }
                }
                // Insert the point at the determined position
                outer_ring.insert(outer_ring.begin() + insert_position, p);
                // Close the ring by adding the first point at the end
                if (outer_ring.size() >= 3){
                    outer_ring.push_back(outer_ring.front());
                }
                // Optional: Correct the polygon to ensure validity (orientation, closure)
                bg::correct(poly);
                // Check if the polygon is valid
                if (!bg::is_valid(poly)){
                    throw std::runtime_error("Polygon is invalid after insertion.");
                }
            }

            // Function to create a connection between two points
            LineString create_connection(const Point& p1, const Point& p2) const {
                LineString connection;
                connection.push_back(p1);
                connection.push_back(p2);
                return connection;
            }

            // Function to generate a unique identifier for each swath
            std::string generate_UUID() const {
                return boost::uuids::to_string(boost::uuids::random_generator()());
            }

            //function that cheks if swath touches perimeter of the field
            bool intersects_field(const LineString& swath, const Field& field) {
                auto edges = field.get_edges();
                for (const auto& edge : edges) {
                    if (bg::intersects(swath, edge)) {
                        return true;
                    }
                }
                return false;
            }
    };

} // namespace farmtrax

#endif // SWATH_HPP
