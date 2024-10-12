#ifndef FARMTRAX_FIELD_HPP
#define FARMTRAX_FIELD_HPP

#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <cmath>

namespace farmtrax {
    class Field {
        private:
            std::vector<geometry_msgs::msg::Point> points_;
            std::string crop_;
            std::string soil_type_;
            double area_;
            double perimeter_;
            geometry_msgs::msg::Point centroid_;
            bool intersects_with_other_field_;

        public:
            Field() {}
            ~Field() {}

            Field(const std::vector<geometry_msgs::msg::Point>& points) {
                init(points);
            }

            Field generateHeadlands(double headlandWidth) const {
                std::vector<geometry_msgs::msg::Point> headlandPoints;

                // Create a new polygon with points offset by the headland width
                for (int i = 0; i < points_.size(); ++i) {
                    geometry_msgs::msg::Point offsetPoint;
                    offsetPoint.x = points_[i].x + headlandWidth * (points_[(i + 1) % points_.size()].x - points_[i].x) / sqrt(pow(points_[(i + 1) % points_.size()].x - points_[i].x, 2) + pow(points_[(i + 1) % points_.size()].y - points_[i].y, 2));
                    offsetPoint.y = points_[i].y + headlandWidth * (points_[(i + 1) % points_.size()].y - points_[i].y) / sqrt(pow(points_[(i + 1) % points_.size()].x - points_[i].x, 2) + pow(points_[(i + 1) % points_.size()].y - points_[i].y, 2));
                    headlandPoints.push_back(offsetPoint);
                }

                // Create a new Field object with the headland points
                Field headlandField(headlandPoints);

                return headlandField;
            }

            // Field generateHeadlands(double headlandWidth) const {
            //     std::vector<geometry_msgs::msg::Point> headlandPoints;
            //     // Create a new polygon with points offset by the headland width
            //     for (long unsigned int i = 0; i < points_.size(); ++i) {
            //         geometry_msgs::msg::Point offsetPoint;
            //         offsetPoint.x = points_[i].x + headlandWidth * (points_[(i + 1) % points_.size()].x - points_[i].x) / sqrt(pow(points_[(i + 1) % points_.size()].x - points_[i].x, 2) + pow(points_[(i + 1) % points_.size()].y - points_[i].y, 2));
            //         offsetPoint.y = points_[i].y + headlandWidth * (points_[(i + 1) % points_.size()].y - points_[i].y) / sqrt(pow(points_[(i + 1) % points_.size()].x - points_[i].x, 2) + pow(points_[(i + 1) % points_.size()].y - points_[i].y, 2));
            //         headlandPoints.push_back(offsetPoint);
            //     }
            //     // Create a new Field object with the headland points
            //     Field headlandField(headlandPoints);
            //     return headlandField;
            // }

            void init(const std::vector<geometry_msgs::msg::Point>& points){
                points_ = points;
                calculateAreaAndPerimeter();
                calculateCentroid();
            }

            geometry_msgs::msg::PolygonStamped getPolygon() const {
                // Create the polygon from the points
                geometry_msgs::msg::PolygonStamped polygon;
                polygon.header.frame_id = "map";
                polygon.header.stamp = rclcpp::Clock().now();
                polygon.polygon.points.resize(points_.size());
                for (long unsigned int i = 0; i < points_.size(); ++i) {
                    polygon.polygon.points[i].x = static_cast<int>(points_[i].x);
                    polygon.polygon.points[i].y = static_cast<int>(points_[i].y);
                }
                return polygon;
            }

            double getArea() const {
                return area_;
            }

            double getPerimeter() const {
                return perimeter_;
            }

            geometry_msgs::msg::Point getCentroid() const {
                return centroid_;
            }

            bool isPointInside(const geometry_msgs::msg::Point& point) const {
                // Ray casting algorithm
                int intersections = 0;
                for (long unsigned int i = 0; i < points_.size(); ++i) {
                    const geometry_msgs::msg::Point& p1 = points_[i];
                    const geometry_msgs::msg::Point& p2 = points_[(i + 1) % points_.size()];
                    if ((p1.y > point.y) != (p2.y > point.y) &&
                        point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x) {
                        intersections++;
                    }
                }
                return intersections % 2 == 1;
            }

            double distanceToNearestBoundary(const geometry_msgs::msg::Point& point) const {
                double min_distance = std::numeric_limits<double>::max();
                for (long unsigned int i = 0; i < points_.size(); ++i) {
                    const geometry_msgs::msg::Point& p1 = points_[i];
                    const geometry_msgs::msg::Point& p2 = points_[(i + 1) % points_.size()];
                    double distance = distanceToLineSegment(point, p1, p2);
                    min_distance = std::min(min_distance, distance);
                }
                return min_distance;
            }

            double distanceToLineSegment(const geometry_msgs::msg::Point& point,
                                        const geometry_msgs::msg::Point& p1,
                                        const geometry_msgs::msg::Point& p2) const {
                // Calculate the distance from a point to a line segment using vector projection
                geometry_msgs::msg::Vector3 v1;
                v1.x = p2.x - p1.x;
                v1.y = p2.y - p1.y;
                geometry_msgs::msg::Vector3 v2;
                v2.x = point.x - p1.x;
                v2.y = point.y - p1.y;
                double projection = (v1.x * v2.x + v1.y * v2.y) / (v1.x * v1.x + v1.y * v1.y);
                geometry_msgs::msg::Vector3 closest_point;
                closest_point.x = p1.x + projection * v1.x;
                closest_point.y = p1.y + projection * v1.y;
                return sqrt(pow(point.x - closest_point.x, 2) + pow(point.y - closest_point.y, 2));
            }

            bool intersectsWith(const Field& other_field) const {
                // Separating axis theorem
                for (long unsigned int i = 0; i < points_.size(); ++i) {
                    const geometry_msgs::msg::Point& p1 = points_[i];
                    const geometry_msgs::msg::Point& p2 = points_[(i + 1) % points_.size()];
                    geometry_msgs::msg::Vector3 axis;
                    axis.x = p2.x - p1.x;
                    axis.y = p2.y - p1.y;
                    double min1 = std::numeric_limits<double>::max();
                    double max1 = std::numeric_limits<double>::min();
                    double min2 = std::numeric_limits<double>::max();
                    double max2 = std::numeric_limits<double>::min();
                    for (long unsigned int j = 0; j < points_.size(); ++j) {
                        double projection = axis.x * points_[j].x + axis.y * points_[j].y;
                        min1 = std::min(min1, projection);
                        max1 = std::max(max1, projection);
                    }
                    for (long unsigned int j = 0; j < other_field.points_.size(); ++j) {
                        double projection = axis.x * other_field.points_[j].x + axis.y * other_field.points_[j].y;
                        min2 = std::min(min2, projection);
                        max2 = std::max(max2, projection);
                    }
                    if (max1 < min2 || min1 > max2) {
                        return false;
                    }
                }

                for (long unsigned int i = 0; i < other_field.points_.size(); ++i) {
                    const geometry_msgs::msg::Point& p1 = other_field.points_[i];
                    const geometry_msgs::msg::Point& p2 = other_field.points_[(i + 1) % other_field.points_.size()];
                    geometry_msgs::msg::Vector3 axis;
                    axis.x = p2.x - p1.x;
                    axis.y = p2.y - p1.y;
                    double min1 = std::numeric_limits<double>::max();
                    double max1 = std::numeric_limits<double>::min();
                    double min2 = std::numeric_limits<double>::max();
                    double max2 = std::numeric_limits<double>::min();
                    for (long unsigned int j = 0; j < points_.size(); ++j) {
                        double projection = axis.x * points_[j].x + axis.y * points_[j].y;
                        min1 = std::min(min1, projection);
                        max1 = std::max(max1, projection);
                    }
                    for (long unsigned int j = 0; j < other_field.points_.size(); ++j) {
                        double projection = axis.x * other_field.points_[j].x + axis.y * other_field.points_[j].y;
                        min2 = std::min(min2, projection);
                        max2 = std::max(max2, projection);
                    }
                    if (max1 < min2 || min1 > max2) {
                        return false;
                    }
                }
                return true;
            }
        private:
            void calculateAreaAndPerimeter() {
                // Calculate the area using the shoelace formula
                area_ = 0.0;
                for (long unsigned int i = 0; i < points_.size(); ++i) {
                    const geometry_msgs::msg::Point& p1 = points_[i];
                    const geometry_msgs::msg::Point& p2 = points_[(i + 1) % points_.size()];
                    area_ += (p1.x * p2.y - p2.x * p1.y) / 2.0;
                }
                area_ = fabs(area_);
                // Calculate the perimeter by summing the distances between points
                perimeter_ = 0.0;
                for (long unsigned int i = 0; i < points_.size(); ++i) {
                    const geometry_msgs::msg::Point& p1 = points_[i];
                    const geometry_msgs::msg::Point& p2 = points_[(i + 1) % points_.size()];
                    perimeter_ += sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
                }
            }

            void calculateCentroid() {
                // Calculate the centroid using the average of the polygon's vertices
                centroid_.x = 0.0;
                centroid_.y = 0.0;
                for (const geometry_msgs::msg::Point& point : points_) {
                    centroid_.x += point.x;
                    centroid_.y += point.y;
                }
                centroid_.x /= points_.size();
                centroid_.y /= points_.size();
            }

        };

    } // namespace farmtrax

#endif // FARMTRAX_FIELD_HPP