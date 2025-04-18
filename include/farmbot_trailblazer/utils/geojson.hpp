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
} // namespace trailblazer::utils
