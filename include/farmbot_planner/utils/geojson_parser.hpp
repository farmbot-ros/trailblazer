// File: geojson_parser.hpp

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <fstream>

namespace geojson {

enum class Type {
    Point,
    MultiPoint,
    LineString,
    MultiLineString,
    Polygon,
    MultiPolygon,
    GeometryCollection,
    Feature,
    FeatureCollection
};

inline Type TypeFromName(const std::string& typeName) {
    if (typeName == "Point") return Type::Point;
    else if (typeName == "MultiPoint") return Type::MultiPoint;
    else if (typeName == "LineString") return Type::LineString;
    else if (typeName == "MultiLineString") return Type::MultiLineString;
    else if (typeName == "Polygon") return Type::Polygon;
    else if (typeName == "MultiPolygon") return Type::MultiPolygon;
    else if (typeName == "GeometryCollection") return Type::GeometryCollection;
    else if (typeName == "Feature") return Type::Feature;
    else if (typeName == "FeatureCollection") return Type::FeatureCollection;
    else throw std::runtime_error("Unknown GeoJSON type: " + typeName);
}

struct Geometry {
    Type type;
    virtual ~Geometry() = default;
};

struct Point : public Geometry {
    std::vector<double> coordinates; // [longitude, latitude, (optional) altitude]
};

struct MultiPoint : public Geometry {
    std::vector<std::vector<double>> coordinates;
};

struct LineString : public Geometry {
    std::vector<std::vector<double>> coordinates;
};

struct MultiLineString : public Geometry {
    std::vector<std::vector<std::vector<double>>> coordinates;
};

struct Polygon : public Geometry {
    std::vector<std::vector<std::vector<double>>> coordinates;
};

struct MultiPolygon : public Geometry {
    std::vector<std::vector<std::vector<std::vector<double>>>> coordinates;
};

struct GeometryCollection : public Geometry {
    std::vector<std::shared_ptr<Geometry>> geometries;
};

struct Feature {
    std::string id;
    std::shared_ptr<Geometry> geometry;
    nlohmann::json properties;
};

struct FeatureCollection {
    std::vector<Feature> features;
};

// Parsing functions will be declared here (definitions follow below)
std::shared_ptr<Geometry> parseGeometry(const nlohmann::json& j);
Feature parseFeature(const nlohmann::json& j);
FeatureCollection parseFeatureCollection(const nlohmann::json& j);
FeatureCollection parseFile(const std::string& filePath);

} // namespace geojson
