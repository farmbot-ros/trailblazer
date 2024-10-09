// File: geojson_parser.cpp

#include "farmbot_planner/utils/geojson_parser.hpp"

namespace geojson {

std::shared_ptr<Point> parsePoint(const nlohmann::json& j) {
    auto point = std::make_shared<Point>();
    point->type = Type::Point;
    point->coordinates = j.at("coordinates").get<std::vector<double>>();
    return point;
}

std::shared_ptr<MultiPoint> parseMultiPoint(const nlohmann::json& j) {
    auto multipoint = std::make_shared<MultiPoint>();
    multipoint->type = Type::MultiPoint;
    multipoint->coordinates = j.at("coordinates").get<std::vector<std::vector<double>>>();
    return multipoint;
}

std::shared_ptr<LineString> parseLineString(const nlohmann::json& j) {
    auto linestring = std::make_shared<LineString>();
    linestring->type = Type::LineString;
    linestring->coordinates = j.at("coordinates").get<std::vector<std::vector<double>>>();
    return linestring;
}

std::shared_ptr<MultiLineString> parseMultiLineString(const nlohmann::json& j) {
    auto multilinestring = std::make_shared<MultiLineString>();
    multilinestring->type = Type::MultiLineString;
    multilinestring->coordinates = j.at("coordinates").get<std::vector<std::vector<std::vector<double>>>>();
    return multilinestring;
}

std::shared_ptr<Polygon> parsePolygon(const nlohmann::json& j) {
    auto polygon = std::make_shared<Polygon>();
    polygon->type = Type::Polygon;
    polygon->coordinates = j.at("coordinates").get<std::vector<std::vector<std::vector<double>>>>();
    return polygon;
}

std::shared_ptr<MultiPolygon> parseMultiPolygon(const nlohmann::json& j) {
    auto multipolygon = std::make_shared<MultiPolygon>();
    multipolygon->type = Type::MultiPolygon;
    multipolygon->coordinates = j.at("coordinates").get<std::vector<std::vector<std::vector<std::vector<double>>>>>();
    return multipolygon;
}

std::shared_ptr<GeometryCollection> parseGeometryCollection(const nlohmann::json& j) {
    auto geometryCollection = std::make_shared<GeometryCollection>();
    geometryCollection->type = Type::GeometryCollection;
    const auto& geometries = j.at("geometries");
    for (const auto& geomJson : geometries) {
        geometryCollection->geometries.push_back(parseGeometry(geomJson));
    }
    return geometryCollection;
}

std::shared_ptr<Geometry> parseGeometry(const nlohmann::json& j) {
    std::string typeStr = j.at("type").get<std::string>();
    Type type = TypeFromName(typeStr);
    switch (type) {
        case Type::Point:
            return parsePoint(j);
        case Type::MultiPoint:
            return parseMultiPoint(j);
        case Type::LineString:
            return parseLineString(j);
        case Type::MultiLineString:
            return parseMultiLineString(j);
        case Type::Polygon:
            return parsePolygon(j);
        case Type::MultiPolygon:
            return parseMultiPolygon(j);
        case Type::GeometryCollection:
            return parseGeometryCollection(j);
        default:
            throw std::runtime_error("Unknown geometry type: " + typeStr);
    }
}

Feature parseFeature(const nlohmann::json& j) {
    Feature feature;
    if (j.contains("id")) {
        feature.id = j["id"].get<std::string>();
    }
    feature.properties = j.value("properties", nlohmann::json{});
    if (j.contains("geometry") && !j["geometry"].is_null()) {
        feature.geometry = parseGeometry(j["geometry"]);
    } else {
        feature.geometry = nullptr; // Geometry can be null
    }
    return feature;
}

FeatureCollection parseFeatureCollection(const nlohmann::json& j) {
    FeatureCollection featureCollection;
    const auto& features = j.at("features");
    for (const auto& featureJson : features) {
        featureCollection.features.push_back(parseFeature(featureJson));
    }
    return featureCollection;
}

nlohmann::json readJSONFromFile(const std::string& filePath) {
    std::ifstream fileStream(filePath);
    if (!fileStream) {
        throw std::runtime_error("Failed to open file: " + filePath);
    }
    nlohmann::json j;
    fileStream >> j;
    return j;
}

FeatureCollection parseFile(const std::string& filePath) {
    nlohmann::json j = readJSONFromFile(filePath);
    return parseFeatureCollection(j);
}

} // namespace geojson
