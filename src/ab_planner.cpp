// src/field_processor_node.cpp

#include "fields2cover.h"
#include <iostream>
#include "farmbot_planner/utils/geojson.hpp"

// GDAL/OGR Headers
#include <ogr_spatialref.h>
#include <ogr_geometry.h>
#include <ogrsf_frmts.h>

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <optional>
#include <cmath>

// Function to convert latitude and longitude to UTM coordinates
std::pair<double, double> latLonToUTM(double lat, double lon, 
    const std::string& datum = "WGS84", 
    const std::optional<std::pair<double, double>>& ref_point = std::nullopt) {
    
    // Create the source spatial reference (WGS84)
    OGRSpatialReference src_srs;
    src_srs.SetWellKnownGeogCS(datum.c_str());

    // Determine the UTM zone from the longitude
    int zone = static_cast<int>((lon + 180) / 6) + 1;
    bool is_northern = lat >= 0;

    // Create the destination spatial reference (UTM)
    OGRSpatialReference dst_srs;
    dst_srs.SetUTM(zone, is_northern);
    dst_srs.SetWellKnownGeogCS(datum.c_str());

    // Create the coordinate transformation
    auto transform = std::unique_ptr<OGRCoordinateTransformation>(
        OGRCreateCoordinateTransformation(&src_srs, &dst_srs)
    );

    if (!transform) {
        throw std::runtime_error("Failed to create coordinate transformation.");
    }

    // Create the point and transform it
    OGRPoint point(lon, lat);
    if (!point.transform(transform.get())) {
        throw std::runtime_error("Failed to transform point.");
    }

    // Get the absolute UTM coordinates (easting, northing)
    double easting = point.getX();
    double northing = point.getY();

    // Adjust the coordinates to be relative to the reference point
    if (ref_point) {
        easting -= ref_point->first;
        northing -= ref_point->second;
    }

    return {easting, northing};
}

class FieldProcessorNode : public rclcpp::Node {
    private:
        std::string geojson_file_;
        
    public:
        FieldProcessorNode() : Node("field_processor_node") {
            // Declare and get parameters
            this->declare_parameter<std::string>("geojson_file", "/home/bresilla/FARMBOT/src/farmbot_planner/config/field.geojson");
            this->get_parameter("geojson_file", geojson_file_);

            // Initialize GDAL/OGR
            OGRRegisterAll();

            // Process the field
            try {
                process_field();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
            }
        }

    private:
        void process_field() {
            std::vector<std::vector<double>> points;
            try {
                auto geojsonObject = geojson::parseGeoJSONFromFile(geojson_file_);
                points = geojson::utils::extractFirstPolygon(geojsonObject);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing GeoJSON: %s", e.what());
                return;
            }

            if (points.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No points found in the GeoJSON file.");
                return;
            }

            // Convert reference point to UTM
            auto ref_utm_coords = latLonToUTM(points[0][1], points[0][0]);

            F2CLinearRing ring;
            for (const auto& point : points) {
                auto transformed_point = latLonToUTM(point[1], point[0], "WGS84", ref_utm_coords);
                ring.addPoint(F2CPoint(transformed_point.first, transformed_point.second));
                RCLCPP_INFO(this->get_logger(), "Transformed Point: [%f, %f]", transformed_point.first, transformed_point.second);
            }
            F2CCells cells(F2CCell{ring});

            F2CRobot robot(2, 15);
            f2c::hg::ConstHL const_hl;
            F2CCells no_hl = const_hl.generateHeadlands(cells, 3.0 * robot.getWidth());

            f2c::sg::BruteForce bf;
            F2CSwaths swaths = bf.generateSwaths(M_PI / 1.32, robot.getCovWidth(), no_hl.getGeometry(0));

            f2c::rp::SnakeOrder snake_sorter;
            swaths = snake_sorter.genSortedSwaths(swaths);

            f2c::pp::PathPlanning path_planner;
            robot.setMinTurningRadius(2);  // m
            f2c::pp::DubinsCurves dubins;
            F2CPath path = path_planner.planPath(robot, swaths, dubins);

            f2c::Visualizer::figure();
            f2c::Visualizer::plot(cells);
            f2c::Visualizer::plot(no_hl);
            f2c::Visualizer::plot(path);
            f2c::Visualizer::figure_size(2400, 2400);
            f2c::Visualizer::save("Tutorial_8_1_UTM.png");

            RCLCPP_INFO(this->get_logger(), "Field processing completed and visualization saved.");
        }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FieldProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
