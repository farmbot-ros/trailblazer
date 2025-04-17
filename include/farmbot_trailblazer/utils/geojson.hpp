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

#include <geoson/geoson.hpp>

namespace trailblazer::utils {
    std::vector<std::vector<double>> extractFirstPolygon(const std::shared_ptr<geoson::GeoJSONObject> &obj) {
        using ObjectType = geoson::GeoJSONObject::ObjectType;
        using GeometryType = geoson::Geometry::Type;

        std::vector<std::vector<double>> coordinates;

        if (obj->objectType() == ObjectType::FeatureCollection) {
            auto featureCollection = std::static_pointer_cast<geoson::FeatureCollection>(obj);
            if (!featureCollection->features().empty()) {
                auto feature = featureCollection->features().front();
                auto geometry = feature->geometry();
                if (geometry && geometry->type() == GeometryType::Polygon) {
                    auto polygon = std::static_pointer_cast<geoson::Polygon>(geometry);
                    // Get the first ring
                    if (!polygon->rings().empty()) {
                        const auto &ring = polygon->rings().front();
                        for (const auto &coord : ring) {
                            std::vector<double> point;
                            point.push_back(coord.y); // Latitude
                            point.push_back(coord.x); // Longitude
                            if (coord.hasZ()) {
                                point.push_back(coord.z); // Altitude
                            }
                            coordinates.push_back(point);
                        }
                    } else {
                        throw std::runtime_error("Polygon has no rings");
                    }
                } else {
                    throw std::runtime_error("First feature does not contain a Polygon geometry");
                }
            } else {
                throw std::runtime_error("FeatureCollection is empty");
            }
        } else if (obj->objectType() == ObjectType::Feature) {
            auto feature = std::static_pointer_cast<geoson::Feature>(obj);
            auto geometry = feature->geometry();
            if (geometry && geometry->type() == GeometryType::Polygon) {
                auto polygon = std::static_pointer_cast<geoson::Polygon>(geometry);
                if (!polygon->rings().empty()) {
                    const auto &ring = polygon->rings().front();
                    for (const auto &coord : ring) {
                        std::vector<double> point;
                        point.push_back(coord.y); // Latitude
                        point.push_back(coord.x); // Longitude
                        if (coord.hasZ()) {
                            point.push_back(coord.z); // Altitude
                        }
                        coordinates.push_back(point);
                    }
                } else {
                    throw std::runtime_error("Polygon has no rings");
                }
            } else {
                throw std::runtime_error("Feature does not contain a Polygon geometry");
            }
        } else if (obj->objectType() == ObjectType::Geometry) {
            auto geometry = std::static_pointer_cast<geoson::Geometry>(obj);
            if (geometry->type() == GeometryType::Polygon) {
                auto polygon = std::static_pointer_cast<geoson::Polygon>(geometry);
                if (!polygon->rings().empty()) {
                    const auto &ring = polygon->rings().front();
                    for (const auto &coord : ring) {
                        std::vector<double> point;
                        point.push_back(coord.y); // Latitude
                        point.push_back(coord.x); // Longitude
                        if (coord.hasZ()) {
                            point.push_back(coord.z); // Altitude
                        }
                        coordinates.push_back(point);
                    }
                } else {
                    throw std::runtime_error("Polygon has no rings");
                }
            } else {
                throw std::runtime_error("Geometry is not a Polygon");
            }
        } else {
            throw std::runtime_error("Unsupported GeoJSONObject type");
        }

        return coordinates;
    }
} // namespace trailblazer::utils
