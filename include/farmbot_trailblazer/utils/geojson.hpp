#pragma once

#include <cmath> // For std::isnan
#include <fstream>
#include <iostream>
#include <limits> // For std::numeric_limits<double>::quiet_NaN()
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "farmbot_interfaces/msg/field.hpp"

namespace trailblazer::utils {
    inline std::vector<std::vector<double>> extractFirstPolygon(const nlohmann::json &fc) {
        // must be a FeatureCollection
        if (fc.value("type", "") != "FeatureCollection")
            throw std::invalid_argument("input is not a FeatureCollection");

        if (fc["features"].empty()) throw std::runtime_error("FeatureCollection is empty");

        const auto &feat = fc["features"].front();
        if (feat.value("type", "") != "Feature") throw std::runtime_error("first element is not a Feature");

        const auto &geom = feat["geometry"];
        if (geom.value("type", "") != "Polygon") throw std::runtime_error("first feature is not a Polygon");

        // first ring of the polygon
        const auto &ring = geom["coordinates"].front();
        if (!ring.is_array() || ring.empty()) throw std::runtime_error("Polygon has no rings");

        std::vector<std::vector<double>> coords;
        coords.reserve(ring.size());

        for (const auto &pt : ring) // pt is [lon, lat] or [lon, lat, alt]
            coords.push_back(pt.get<std::vector<double>>());

        // convert to [lat, lon]
        for (auto &pt : coords) {
            std::swap(pt[0], pt[1]);
        }

        return coords;
    }

    inline nlohmann::json ReadFeatureCollection(const std::filesystem::path &file) {
        std::ifstream ifs(file);
        if (!ifs) throw std::runtime_error("ReadFeatureCollection(): cannot open \"" + file.string() + '\"');

        nlohmann::json j;
        ifs >> j;

        if (!j.is_object() || !j.contains("type") || !j["type"].is_string())
            throw std::runtime_error("ReadFeatureCollection(): top‑level object has no string 'type' field");

        const std::string type = j["type"].get<std::string>();

        if (type == "FeatureCollection") return j; // as‑is

        if (type == "Feature") // wrap Feature
            return nlohmann::json{{"type", "FeatureCollection"}, {"features", nlohmann::json::array({j})}};

        // otherwise treat it as a bare geometry --------------------------------
        nlohmann::json feature{
            {"type", "Feature"}, {"geometry", j}, {"properties", nlohmann::json::object()}}; // empty props

        return nlohmann::json{{"type", "FeatureCollection"}, {"features", nlohmann::json::array({feature})}};
    }

    inline nlohmann::json colleciton_from_field(const farmbot_interfaces::msg::Field &field) {
        nlohmann::json gs;
        gs["type"] = "FeatureCollection";
        gs["features"] = nlohmann::json::array();
        for (const auto &swath : field.swaths.lines) {
            std::string uuid = swath.uuid;
            nlohmann::json af;
            af["type"] = "Feature";
            af["properties"] = {{"uuid", uuid}, {"type", "swath"}};
            af["geometry"] = nlohmann::json::object();
            af["geometry"]["type"] = "LineString";
            af["geometry"]["coordinates"] = nlohmann::json::array();
            auto lat_0 = swath.geo_line[0].x;
            auto lon_0 = swath.geo_line[0].y;
            auto lat_1 = swath.geo_line[1].x;
            auto lon_1 = swath.geo_line[1].y;
            af["geometry"]["coordinates"].push_back({lon_0, lat_0});
            af["geometry"]["coordinates"].push_back({lon_1, lat_1});
            gs["features"].push_back(af);
        }
        std::string uuid = field.border.lines[0].uuid;
        nlohmann::json ac;
        ac["type"] = "Feature";
        ac["properties"] = {{"uuid", uuid}, {"type", "border"}};
        ac["geometry"] = nlohmann::json::object();
        ac["geometry"]["type"] = "Polygon";
        ac["geometry"]["coordinates"] = nlohmann::json::array();
        for (const auto &border : field.border.lines) {
            auto lat = border.geo_line[0].x;
            auto lon = border.geo_line[0].y;
            ac["geometry"]["coordinates"][0].push_back({lon, lat});
        }
        gs["features"].push_back(ac);
        return gs;
    }

    inline farmbot_interfaces::msg::Field field_from_collection(const nlohmann::json &collection) {
        farmbot_interfaces::msg::Field field;
        // Process all features in the collection
        for (const auto &feature : collection["features"]) {
            // Check feature properties
            const auto &properties = feature["properties"];
            const auto &geometry = feature["geometry"];
            const std::string type = properties["type"];
            const std::string uuid = properties["uuid"];

            if (type == "swath") {
                // Process swath lines
                farmbot_interfaces::msg::Line line;
                line.uuid = uuid;

                // Get coordinates
                const auto &coords = geometry["coordinates"];
                if (coords.size() >= 2) {
                    // First point
                    geometry_msgs::msg::Point p1;
                    p1.x = coords[0][1]; // lat (swap from lon,lat to x,y)
                    p1.y = coords[0][0]; // lon
                    p1.z = 0.0;

                    // Second point
                    geometry_msgs::msg::Point p2;
                    p2.x = coords[1][1]; // lat
                    p2.y = coords[1][0]; // lon
                    p2.z = 0.0;

                    line.geo_line = {p1, p2};
                    field.swaths.lines.push_back(line);
                }
            } else if (type == "border") {
                // Process border polygon
                farmbot_interfaces::msg::Line border_line;
                border_line.uuid = uuid;

                // Get polygon coordinates (first ring)
                const auto &polygon_coords = geometry["coordinates"][0];

                // Create border points
                for (const auto &coord : polygon_coords) {
                    geometry_msgs::msg::Point point;
                    point.x = coord[1]; // lat (swap from lon,lat to x,y)
                    point.y = coord[0]; // lon
                    point.z = 0.0;

                    farmbot_interfaces::msg::Line line;
                    line.uuid = uuid;
                    line.geo_line = {point};
                    field.border.lines.push_back(line);
                }
            }
        }
        return field;
    }

    // Convenience function to read from file
    inline farmbot_interfaces::msg::Field field_from_geojson_file(const std::filesystem::path &file) {
        auto collection = ReadFeatureCollection(file);
        return field_from_collection(collection);
    }
} // namespace trailblazer::utils
