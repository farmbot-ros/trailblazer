#ifndef GEOJSON_PARSER_HPP
#define GEOJSON_PARSER_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <fstream> 
#include <iostream>
#include <cmath>    // For std::isnan
#include <limits>   // For std::numeric_limits<double>::quiet_NaN()

namespace geojson {

    // Forward declarations
    class GeoJSONObject;
    class Geometry;
    class Point;
    class MultiPoint;
    class LineString;
    class MultiLineString;
    class Polygon;
    class MultiPolygon;
    class GeometryCollection;
    class Feature;
    class FeatureCollection;

    // Coordinate struct
    struct Coordinate {
        double x;
        double y;
        double z; // Optional z-coordinate, NaN if not present

        Coordinate(double x_, double y_) : x(x_), y(y_), z(std::numeric_limits<double>::quiet_NaN()) {}
        Coordinate(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

        bool hasZ() const {
            return !std::isnan(z);
        }
    };

    // JSON Value class
    class Value {
    public:
        enum class Type {
            Null,
            Boolean,
            Number,
            String,
            Array,
            Object
        };

        // Constructors
        Value() : _type(Type::Null) {}
        Value(bool b) : _type(Type::Boolean), _bool_value(b) {}
        Value(double n) : _type(Type::Number), _number_value(n) {}
        Value(const std::string& s) : _type(Type::String), _string_value(s) {}
        Value(const std::vector<Value>& a) : _type(Type::Array), _array_value(a) {}
        Value(const std::map<std::string, Value>& o) : _type(Type::Object), _object_value(o) {}

        // Accessors
        Type type() const { return _type; }

        bool asBool() const {
            if (_type != Type::Boolean) throw std::runtime_error("Value is not a boolean");
            return _bool_value;
        }

        double asNumber() const {
            if (_type != Type::Number) throw std::runtime_error("Value is not a number");
            return _number_value;
        }

        const std::string& asString() const {
            if (_type != Type::String) throw std::runtime_error("Value is not a string");
            return _string_value;
        }

        const std::vector<Value>& asArray() const {
            if (_type != Type::Array) throw std::runtime_error("Value is not an array");
            return _array_value;
        }

        const std::map<std::string, Value>& asObject() const {
            if (_type != Type::Object) throw std::runtime_error("Value is not an object");
            return _object_value;
        }

        // Type checkers
        bool isNull() const { return _type == Type::Null; }
        bool isBoolean() const { return _type == Type::Boolean; }
        bool isNumber() const { return _type == Type::Number; }
        bool isString() const { return _type == Type::String; }
        bool isArray() const { return _type == Type::Array; }
        bool isObject() const { return _type == Type::Object; }

    private:
        Type _type;
        bool _bool_value;
        double _number_value;
        std::string _string_value;
        std::vector<Value> _array_value;
        std::map<std::string, Value> _object_value;
    };

    // JSON Parser class
    class JSONParser {
    public:
        JSONParser(const std::string& input) : _input(input), _pos(0) {}

        Value parse() {
            skipWhitespace();
            Value v = parseValue();
            skipWhitespace();
            if (_pos != _input.length()) {
                throw std::runtime_error("Unexpected character");
            }
            return v;
        }

    private:
        // Parsing methods
        Value parseValue() {
            skipWhitespace();
            if (_pos >= _input.length()) {
                throw std::runtime_error("Unexpected end of input");
            }

            char ch = peek();
            if (ch == 'n') {
                return parseNull();
            } else if (ch == 't' || ch == 'f') {
                return parseBool();
            } else if (ch == '"' || ch == '\'') {
                return parseString();
            } else if (ch == '-' || (ch >= '0' && ch <= '9')) {
                return parseNumber();
            } else if (ch == '[') {
                return parseArray();
            } else if (ch == '{') {
                return parseObject();
            } else {
                throw std::runtime_error(std::string("Unexpected character: ") + ch);
            }
        }

        Value parseNull() {
            expect('n');
            if (_input.substr(_pos, 3) != "ull") {
                throw std::runtime_error("Invalid token");
            }
            _pos += 3;
            return Value();
        }

        Value parseBool() {
            char ch = peek();
            if (ch == 't') {
                expect('t');
                if (_input.substr(_pos, 3) != "rue") {
                    throw std::runtime_error("Invalid token");
                }
                _pos += 3;
                return Value(true);
            } else if (ch == 'f') {
                expect('f');
                if (_input.substr(_pos, 4) != "alse") {
                    throw std::runtime_error("Invalid token");
                }
                _pos += 4;
                return Value(false);
            } else {
                throw std::runtime_error("Invalid boolean value");
            }
        }

        Value parseNumber() {
            size_t start_pos = _pos;

            if (peek() == '-') {
                get();
            }

            if (peek() == '0') {
                get();
            } else if (peek() >= '1' && peek() <= '9') {
                while (peek() >= '0' && peek() <= '9') {
                    get();
                }
            } else {
                throw std::runtime_error("Invalid number");
            }

            if (peek() == '.') {
                get();
                if (peek() < '0' || peek() > '9') {
                    throw std::runtime_error("Invalid number");
                }
                while (peek() >= '0' && peek() <= '9') {
                    get();
                }
            }

            if (peek() == 'e' || peek() == 'E') {
                get();
                if (peek() == '+' || peek() == '-') {
                    get();
                }
                if (peek() < '0' || peek() > '9') {
                    throw std::runtime_error("Invalid number");
                }
                while (peek() >= '0' && peek() <= '9') {
                    get();
                }
            }

            std::string number_str = _input.substr(start_pos, _pos - start_pos);
            double number = std::stod(number_str);
            return Value(number);
        }

        Value parseString() {
            std::string str = parseStringLiteral();
            return Value(str);
        }

        std::string parseStringLiteral() {
            char quote = get(); // consume the opening quote
            if (quote != '"' && quote != '\'') {
                throw std::runtime_error("Expected opening quote for string");
            }

            std::string result;
            while (true) {
                if (_pos >= _input.length()) {
                    throw std::runtime_error("Unterminated string");
                }
                char ch = get();
                if (ch == quote) {
                    break;
                }
                if (ch == '\\') {
                    if (_pos >= _input.length()) {
                        throw std::runtime_error("Unterminated string");
                    }
                    ch = get();
                    if (ch == '\"') result += '\"';
                    else if (ch == '\\') result += '\\';
                    else if (ch == '/') result += '/';
                    else if (ch == 'b') result += '\b';
                    else if (ch == 'f') result += '\f';
                    else if (ch == 'n') result += '\n';
                    else if (ch == 'r') result += '\r';
                    else if (ch == 't') result += '\t';
                    else if (ch == 'u') {
                        // Unicode escape sequence
                        if (_pos + 4 > _input.length()) {
                            throw std::runtime_error("Invalid unicode escape sequence");
                        }
                        std::string hex = _input.substr(_pos, 4);
                        _pos += 4;
                        char16_t code = static_cast<char16_t>(std::stoi(hex, nullptr, 16));
                        // For simplicity, only handle code points in the ASCII range
                        result += static_cast<char>(code);
                    } else {
                        throw std::runtime_error(std::string("Invalid escape sequence: \\") + ch);
                    }
                } else {
                    result += ch;
                }
            }
            return result;
        }

        Value parseArray() {
            expect('[');
            skipWhitespace();
            std::vector<Value> array;

            if (peek() == ']') {
                get();
                return Value(array);
            }

            while (true) {
                Value v = parseValue();
                array.push_back(v);
                skipWhitespace();
                if (peek() == ',') {
                    get();
                    skipWhitespace();
                } else if (peek() == ']') {
                    get();
                    break;
                } else {
                    throw std::runtime_error("Expected ',' or ']' in array");
                }
            }
            return Value(array);
        }

        Value parseObject() {
            expect('{');
            skipWhitespace();
            std::map<std::string, Value> object;

            if (peek() == '}') {
                get();
                return Value(object);
            }

            while (true) {
                skipWhitespace();
                if (peek() != '"' && peek() != '\'') {
                    throw std::runtime_error("Expected string key in object");
                }
                std::string key = parseStringLiteral();
                skipWhitespace();
                expect(':');
                skipWhitespace();
                Value value = parseValue();
                object[key] = value;
                skipWhitespace();
                if (peek() == ',') {
                    get();
                    skipWhitespace();
                } else if (peek() == '}') {
                    get();
                    break;
                } else {
                    throw std::runtime_error("Expected ',' or '}' in object");
                }
            }
            return Value(object);
        }

        // Helper methods
        char peek() const {
            if (_pos >= _input.length()) {
                return '\0';
            }
            return _input[_pos];
        }

        char get() {
            if (_pos >= _input.length()) {
                throw std::runtime_error("Unexpected end of input");
            }
            return _input[_pos++];
        }

        void skipWhitespace() {
            while (_pos < _input.length()) {
                char ch = _input[_pos];
                if (ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r') {
                    _pos++;
                } else {
                    break;
                }
            }
        }

        void expect(char ch) {
            if (get() != ch) {
                throw std::runtime_error(std::string("Expected '") + ch + "'");
            }
        }

        const std::string& _input;
        size_t _pos;
    };

    // Base class for all GeoJSON objects
    class GeoJSONObject {
    public:
        enum class ObjectType {
            Geometry,
            Feature,
            FeatureCollection
        };

        virtual ~GeoJSONObject() = default;
        virtual ObjectType objectType() const = 0;
    };

    // Geometry base class
    class Geometry : public GeoJSONObject {
    public:
        enum class Type {
            Point,
            MultiPoint,
            LineString,
            MultiLineString,
            Polygon,
            MultiPolygon,
            GeometryCollection
        };

        virtual ~Geometry() = default;
        virtual Type type() const = 0;

        ObjectType objectType() const override { return ObjectType::Geometry; }
    };

    // Point class
    class Point : public Geometry {
    public:
        Point(const Coordinate& coord) : _coordinate(coord) {}

        Type type() const override { return Type::Point; }

        const Coordinate& coordinate() const { return _coordinate; }

    private:
        Coordinate _coordinate;
    };

    // MultiPoint class
    class MultiPoint : public Geometry {
    public:
        MultiPoint(const std::vector<Coordinate>& points) : _points(points) {}

        Type type() const override { return Type::MultiPoint; }

        const std::vector<Coordinate>& points() const { return _points; }

    private:
        std::vector<Coordinate> _points;
    };

    // LineString class
    class LineString : public Geometry {
    public:
        LineString(const std::vector<Coordinate>& points) : _points(points) {}

        Type type() const override { return Type::LineString; }

        const std::vector<Coordinate>& points() const { return _points; }

    private:
        std::vector<Coordinate> _points;
    };

    // MultiLineString class
    class MultiLineString : public Geometry {
    public:
        MultiLineString(const std::vector<std::vector<Coordinate>>& lines) : _lines(lines) {}

        Type type() const override { return Type::MultiLineString; }

        const std::vector<std::vector<Coordinate>>& lines() const { return _lines; }

    private:
        std::vector<std::vector<Coordinate>> _lines;
    };

    // Polygon class
    class Polygon : public Geometry {
    public:
        Polygon(const std::vector<std::vector<Coordinate>>& rings) : _rings(rings) {}

        Type type() const override { return Type::Polygon; }

        const std::vector<std::vector<Coordinate>>& rings() const { return _rings; }

    private:
        std::vector<std::vector<Coordinate>> _rings;
    };

    // MultiPolygon class
    class MultiPolygon : public Geometry {
    public:
        MultiPolygon(const std::vector<std::vector<std::vector<Coordinate>>>& polygons) : _polygons(polygons) {}

        Type type() const override { return Type::MultiPolygon; }

        const std::vector<std::vector<std::vector<Coordinate>>>& polygons() const { return _polygons; }

    private:
        std::vector<std::vector<std::vector<Coordinate>>> _polygons;
    };

    // GeometryCollection class
    class GeometryCollection : public Geometry {
    public:
        GeometryCollection(const std::vector<std::shared_ptr<Geometry>>& geometries) : _geometries(geometries) {}

        Type type() const override { return Type::GeometryCollection; }

        const std::vector<std::shared_ptr<Geometry>>& geometries() const { return _geometries; }

    private:
        std::vector<std::shared_ptr<Geometry>> _geometries;
    };

    // Feature class
    class Feature : public GeoJSONObject {
    public:
        Feature(std::shared_ptr<Geometry> geometry,
                const std::map<std::string, Value>& properties,
                const Value& id = Value())
            : _geometry(geometry), _properties(properties), _id(id) {}

        std::shared_ptr<Geometry> geometry() const { return _geometry; }
        const std::map<std::string, Value>& properties() const { return _properties; }
        const Value& id() const { return _id; }

        ObjectType objectType() const override { return ObjectType::Feature; }

    private:
        std::shared_ptr<Geometry> _geometry;
        std::map<std::string, Value> _properties;
        Value _id; // Optional
    };

    // FeatureCollection class
    class FeatureCollection : public GeoJSONObject {
    public:
        FeatureCollection(const std::vector<std::shared_ptr<Feature>>& features)
            : _features(features) {}

        const std::vector<std::shared_ptr<Feature>>& features() const { return _features; }

        ObjectType objectType() const override { return ObjectType::FeatureCollection; }

    private:
        std::vector<std::shared_ptr<Feature>> _features;
    };

    // Parsing functions
    inline std::shared_ptr<Geometry> parseGeometry(const Value& value);

    inline std::shared_ptr<Point> parsePoint(const Value& coordinates) {
        if (!coordinates.isArray()) {
            throw std::runtime_error("Point coordinates must be an array");
        }
        const auto& coords = coordinates.asArray();
        if (coords.size() < 2) {
            throw std::runtime_error("Point coordinates must have at least two elements");
        }
        double x = coords[0].asNumber();
        double y = coords[1].asNumber();
        if (coords.size() >= 3) {
            double z = coords[2].asNumber();
            Coordinate coord(x, y, z);
            return std::make_shared<Point>(coord);
        } else {
            Coordinate coord(x, y);
            return std::make_shared<Point>(coord);
        }
    }

    inline std::shared_ptr<MultiPoint> parseMultiPoint(const Value& coordinates) {
        if (!coordinates.isArray()) {
            throw std::runtime_error("MultiPoint coordinates must be an array");
        }
        const auto& coords_array = coordinates.asArray();
        std::vector<Coordinate> points;
        for (const auto& coord_value : coords_array) {
            if (!coord_value.isArray()) {
                throw std::runtime_error("Coordinate must be an array");
            }
            const auto& coord = coord_value.asArray();
            if (coord.size() < 2) {
                throw std::runtime_error("Coordinate must have at least two elements");
            }
            double x = coord[0].asNumber();
            double y = coord[1].asNumber();
            if (coord.size() >= 3) {
                double z = coord[2].asNumber();
                points.emplace_back(x, y, z);
            } else {
                points.emplace_back(x, y);
            }
        }
        return std::make_shared<MultiPoint>(points);
    }

    inline std::shared_ptr<LineString> parseLineString(const Value& coordinates) {
        if (!coordinates.isArray()) {
            throw std::runtime_error("LineString coordinates must be an array");
        }
        const auto& coords_array = coordinates.asArray();
        std::vector<Coordinate> points;
        for (const auto& coord_value : coords_array) {
            if (!coord_value.isArray()) {
                throw std::runtime_error("Coordinate must be an array");
            }
            const auto& coord = coord_value.asArray();
            if (coord.size() < 2) {
                throw std::runtime_error("Coordinate must have at least two elements");
            }
            double x = coord[0].asNumber();
            double y = coord[1].asNumber();
            if (coord.size() >= 3) {
                double z = coord[2].asNumber();
                points.emplace_back(x, y, z);
            } else {
                points.emplace_back(x, y);
            }
        }
        return std::make_shared<LineString>(points);
    }

    inline std::shared_ptr<MultiLineString> parseMultiLineString(const Value& coordinates) {
        if (!coordinates.isArray()) {
            throw std::runtime_error("MultiLineString coordinates must be an array");
        }
        const auto& lines_array = coordinates.asArray();
        std::vector<std::vector<Coordinate>> lines;
        for (const auto& line_value : lines_array) {
            if (!line_value.isArray()) {
                throw std::runtime_error("Line must be an array");
            }
            const auto& coords_array = line_value.asArray();
            std::vector<Coordinate> points;
            for (const auto& coord_value : coords_array) {
                if (!coord_value.isArray()) {
                    throw std::runtime_error("Coordinate must be an array");
                }
                const auto& coord = coord_value.asArray();
                if (coord.size() < 2) {
                    throw std::runtime_error("Coordinate must have at least two elements");
                }
                double x = coord[0].asNumber();
                double y = coord[1].asNumber();
                if (coord.size() >= 3) {
                    double z = coord[2].asNumber();
                    points.emplace_back(x, y, z);
                } else {
                    points.emplace_back(x, y);
                }
            }
            lines.push_back(points);
        }
        return std::make_shared<MultiLineString>(lines);
    }

    inline std::shared_ptr<Polygon> parsePolygon(const Value& coordinates) {
        if (!coordinates.isArray()) {
            throw std::runtime_error("Polygon coordinates must be an array");
        }
        const auto& rings_array = coordinates.asArray();
        std::vector<std::vector<Coordinate>> rings;
        for (const auto& ring_value : rings_array) {
            if (!ring_value.isArray()) {
                throw std::runtime_error("Ring must be an array");
            }
            const auto& coords_array = ring_value.asArray();
            std::vector<Coordinate> points;
            for (const auto& coord_value : coords_array) {
                if (!coord_value.isArray()) {
                    throw std::runtime_error("Coordinate must be an array");
                }
                const auto& coord = coord_value.asArray();
                if (coord.size() < 2) {
                    throw std::runtime_error("Coordinate must have at least two elements");
                }
                double x = coord[0].asNumber();
                double y = coord[1].asNumber();
                if (coord.size() >= 3) {
                    double z = coord[2].asNumber();
                    points.emplace_back(x, y, z);
                } else {
                    points.emplace_back(x, y);
                }
            }
            rings.push_back(points);
        }
        return std::make_shared<Polygon>(rings);
    }

    inline std::shared_ptr<MultiPolygon> parseMultiPolygon(const Value& coordinates) {
        if (!coordinates.isArray()) {
            throw std::runtime_error("MultiPolygon coordinates must be an array");
        }
        const auto& polygons_array = coordinates.asArray();
        std::vector<std::vector<std::vector<Coordinate>>> polygons;
        for (const auto& polygon_value : polygons_array) {
            if (!polygon_value.isArray()) {
                throw std::runtime_error("Polygon must be an array");
            }
            const auto& rings_array = polygon_value.asArray();
            std::vector<std::vector<Coordinate>> rings;
            for (const auto& ring_value : rings_array) {
                if (!ring_value.isArray()) {
                    throw std::runtime_error("Ring must be an array");
                }
                const auto& coords_array = ring_value.asArray();
                std::vector<Coordinate> points;
                for (const auto& coord_value : coords_array) {
                    if (!coord_value.isArray()) {
                        throw std::runtime_error("Coordinate must be an array");
                    }
                    const auto& coord = coord_value.asArray();
                    if (coord.size() < 2) {
                        throw std::runtime_error("Coordinate must have at least two elements");
                    }
                    double x = coord[0].asNumber();
                    double y = coord[1].asNumber();
                    if (coord.size() >= 3) {
                        double z = coord[2].asNumber();
                        points.emplace_back(x, y, z);
                    } else {
                        points.emplace_back(x, y);
                    }
                }
                rings.push_back(points);
            }
            polygons.push_back(rings);
        }
        return std::make_shared<MultiPolygon>(polygons);
    }

    inline std::shared_ptr<GeometryCollection> parseGeometryCollection(const Value& geometries) {
        if (!geometries.isArray()) {
            throw std::runtime_error("GeometryCollection geometries must be an array");
        }
        const auto& geometries_array = geometries.asArray();
        std::vector<std::shared_ptr<Geometry>> geometry_list;
        for (const auto& geom_value : geometries_array) {
            std::shared_ptr<Geometry> geom = parseGeometry(geom_value);
            geometry_list.push_back(geom);
        }
        return std::make_shared<GeometryCollection>(geometry_list);
    }

    inline std::shared_ptr<Geometry> parseGeometry(const Value& value) {
        if (!value.isObject()) {
            throw std::runtime_error("Invalid Geometry: Not an object");
        }

        const auto& obj = value.asObject();

        auto it_type = obj.find("type");
        if (it_type == obj.end() || !it_type->second.isString()) {
            throw std::runtime_error("Invalid Geometry: Missing or invalid 'type' property");
        }

        std::string type = it_type->second.asString();

        if (type == "Point") {
            auto it_coordinates = obj.find("coordinates");
            if (it_coordinates == obj.end()) {
                throw std::runtime_error("Point geometry missing 'coordinates'");
            }
            return parsePoint(it_coordinates->second);
        } else if (type == "MultiPoint") {
            auto it_coordinates = obj.find("coordinates");
            if (it_coordinates == obj.end()) {
                throw std::runtime_error("MultiPoint geometry missing 'coordinates'");
            }
            return parseMultiPoint(it_coordinates->second);
        } else if (type == "LineString") {
            auto it_coordinates = obj.find("coordinates");
            if (it_coordinates == obj.end()) {
                throw std::runtime_error("LineString geometry missing 'coordinates'");
            }
            return parseLineString(it_coordinates->second);
        } else if (type == "MultiLineString") {
            auto it_coordinates = obj.find("coordinates");
            if (it_coordinates == obj.end()) {
                throw std::runtime_error("MultiLineString geometry missing 'coordinates'");
            }
            return parseMultiLineString(it_coordinates->second);
        } else if (type == "Polygon") {
            auto it_coordinates = obj.find("coordinates");
            if (it_coordinates == obj.end()) {
                throw std::runtime_error("Polygon geometry missing 'coordinates'");
            }
            return parsePolygon(it_coordinates->second);
        } else if (type == "MultiPolygon") {
            auto it_coordinates = obj.find("coordinates");
            if (it_coordinates == obj.end()) {
                throw std::runtime_error("MultiPolygon geometry missing 'coordinates'");
            }
            return parseMultiPolygon(it_coordinates->second);
        } else if (type == "GeometryCollection") {
            auto it_geometries = obj.find("geometries");
            if (it_geometries == obj.end()) {
                throw std::runtime_error("GeometryCollection missing 'geometries'");
            }
            return parseGeometryCollection(it_geometries->second);
        } else {
            throw std::runtime_error("Unknown geometry type: " + type);
        }
    }

    // Parsing function for Feature
    inline std::shared_ptr<Feature> parseFeature(const Value& value) {
        if (!value.isObject()) {
            throw std::runtime_error("Invalid Feature: Not an object");
        }

        const auto& obj = value.asObject();

        // Get "type"
        auto it_type = obj.find("type");
        if (it_type == obj.end() || !it_type->second.isString()) {
            throw std::runtime_error("Invalid Feature: Missing or invalid 'type' property");
        }
        if (it_type->second.asString() != "Feature") {
            throw std::runtime_error("Invalid Feature: 'type' must be 'Feature'");
        }

        // Get "geometry"
        auto it_geometry = obj.find("geometry");
        if (it_geometry == obj.end() || (!it_geometry->second.isObject() && !it_geometry->second.isNull())) {
            throw std::runtime_error("Invalid Feature: Missing or invalid 'geometry' property");
        }

        std::shared_ptr<Geometry> geometry;
        if (!it_geometry->second.isNull()) {
            geometry = parseGeometry(it_geometry->second);
        }

        // Get "properties"
        auto it_properties = obj.find("properties");
        if (it_properties == obj.end() || !it_properties->second.isObject()) {
            throw std::runtime_error("Invalid Feature: Missing or invalid 'properties' property");
        }
        const auto& properties = it_properties->second.asObject();

        // Get "id" (optional)
        Value id;
        auto it_id = obj.find("id");
        if (it_id != obj.end()) {
            id = it_id->second;
        }

        return std::make_shared<Feature>(geometry, properties, id);
    }

    // Parsing function for FeatureCollection
    inline std::shared_ptr<FeatureCollection> parseFeatureCollection(const Value& value) {
        if (!value.isObject()) {
            throw std::runtime_error("Invalid FeatureCollection: Not an object");
        }

        const auto& obj = value.asObject();

        // Get "type"
        auto it_type = obj.find("type");
        if (it_type == obj.end() || !it_type->second.isString()) {
            throw std::runtime_error("Invalid FeatureCollection: Missing or invalid 'type' property");
        }
        if (it_type->second.asString() != "FeatureCollection") {
            throw std::runtime_error("Invalid FeatureCollection: 'type' must be 'FeatureCollection'");
        }

        // Get "features"
        auto it_features = obj.find("features");
        if (it_features == obj.end() || !it_features->second.isArray()) {
            throw std::runtime_error("Invalid FeatureCollection: Missing or invalid 'features' property");
        }
        const auto& features_array = it_features->second.asArray();

        std::vector<std::shared_ptr<Feature>> features;
        for (const auto& feature_value : features_array) {
            std::shared_ptr<Feature> feature = parseFeature(feature_value);
            features.push_back(feature);
        }

        return std::make_shared<FeatureCollection>(features);
    }

    // Function to parse GeoJSON string into GeoJSONObject
    inline std::shared_ptr<GeoJSONObject> parseGeoJSON(const std::string& jsonStr) {
        JSONParser parser(jsonStr);
        Value root = parser.parse();

        if (!root.isObject()) {
            throw std::runtime_error("Invalid GeoJSON: Root element is not an object");
        }

        const auto& obj = root.asObject();

        auto it_type = obj.find("type");
        if (it_type == obj.end() || !it_type->second.isString()) {
            throw std::runtime_error("Invalid GeoJSON: Missing or invalid 'type' property");
        }

        std::string type = it_type->second.asString();

        if (type == "Feature") {
            // Parse Feature
            return parseFeature(root);
        } else if (type == "FeatureCollection") {
            // Parse FeatureCollection
            return parseFeatureCollection(root);
        } else {
            // Parse as Geometry
            return parseGeometry(root);
        }
    }

    // Function to read a file's contents into a string
    inline std::string readFileContents(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        std::ostringstream oss;
        oss << file.rdbuf();
        return oss.str();
    }

    // Function to parse GeoJSON from a file, returning GeoJSONObject
    inline std::shared_ptr<GeoJSONObject> parseGeoJSONFromFile(const std::string& filename) {
        std::string jsonStr = readFileContents(filename);
        return parseGeoJSON(jsonStr);
    }

    // Function to print Geometry
    void printGeometry(const std::shared_ptr<geojson::Geometry>& geometry) {
        using Type = geojson::Geometry::Type;

        switch (geometry->type()) {
            case Type::Point: {
                auto point = std::static_pointer_cast<geojson::Point>(geometry);
                const auto& coord = point->coordinate();
                std::cout << "Point: (" << coord.x << ", " << coord.y;
                if (coord.hasZ()) {
                    std::cout << ", " << coord.z;
                }
                std::cout << ")" << std::endl;
                break;
            }
            case Type::MultiPoint: {
                auto multipoint = std::static_pointer_cast<geojson::MultiPoint>(geometry);
                std::cout << "MultiPoint:" << std::endl;
                for (const auto& coord : multipoint->points()) {
                    std::cout << "(" << coord.x << ", " << coord.y;
                    if (coord.hasZ()) {
                        std::cout << ", " << coord.z;
                    }
                    std::cout << ")" << std::endl;
                }
                break;
            }
            case Type::LineString: {
                auto line = std::static_pointer_cast<geojson::LineString>(geometry);
                std::cout << "LineString:" << std::endl;
                for (const auto& coord : line->points()) {
                    std::cout << "(" << coord.x << ", " << coord.y;
                    if (coord.hasZ()) {
                        std::cout << ", " << coord.z;
                    }
                    std::cout << ")" << std::endl;
                }
                break;
            }
            case Type::MultiLineString: {
                auto multiline = std::static_pointer_cast<geojson::MultiLineString>(geometry);
                std::cout << "MultiLineString:" << std::endl;
                int lineIndex = 0;
                for (const auto& line : multiline->lines()) {
                    std::cout << "Line " << lineIndex++ << ":" << std::endl;
                    for (const auto& coord : line) {
                        std::cout << "(" << coord.x << ", " << coord.y;
                        if (coord.hasZ()) {
                            std::cout << ", " << coord.z;
                        }
                        std::cout << ")" << std::endl;
                    }
                }
                break;
            }
            case Type::Polygon: {
                auto polygon = std::static_pointer_cast<geojson::Polygon>(geometry);
                std::cout << "Polygon:" << std::endl;
                int ringIndex = 0;
                for (const auto& ring : polygon->rings()) {
                    std::cout << "Ring " << ringIndex++ << ":" << std::endl;
                    for (const auto& coord : ring) {
                        std::cout << "(" << coord.x << ", " << coord.y;
                        if (coord.hasZ()) {
                            std::cout << ", " << coord.z;
                        }
                        std::cout << ")" << std::endl;
                    }
                }
                break;
            }
            case Type::MultiPolygon: {
                auto multipolygon = std::static_pointer_cast<geojson::MultiPolygon>(geometry);
                std::cout << "MultiPolygon:" << std::endl;
                int polyIndex = 0;
                for (const auto& polygon : multipolygon->polygons()) {
                    std::cout << "Polygon " << polyIndex++ << ":" << std::endl;
                    int ringIndex = 0;
                    for (const auto& ring : polygon) {
                        std::cout << "  Ring " << ringIndex++ << ":" << std::endl;
                        for (const auto& coord : ring) {
                            std::cout << "    (" << coord.x << ", " << coord.y;
                            if (coord.hasZ()) {
                                std::cout << ", " << coord.z;
                            }
                            std::cout << ")" << std::endl;
                        }
                    }
                }
                break;
            }
            case Type::GeometryCollection: {
                auto collection = std::static_pointer_cast<geojson::GeometryCollection>(geometry);
                std::cout << "GeometryCollection:" << std::endl;
                for (const auto& geom : collection->geometries()) {
                    printGeometry(geom);
                }
                break;
            }
            default:
                std::cout << "Unsupported geometry type." << std::endl;
        }
    }

    // Function to print GeoJSONObject
    void printGeoJSONObject(const std::shared_ptr<GeoJSONObject>& obj) {
        using ObjectType = GeoJSONObject::ObjectType;

        switch (obj->objectType()) {
            case ObjectType::Geometry: {
                auto geometry = std::static_pointer_cast<Geometry>(obj);
                printGeometry(geometry);
                break;
            }
            case ObjectType::Feature: {
                auto feature = std::static_pointer_cast<Feature>(obj);
                std::cout << "Feature:" << std::endl;
                if (!feature->id().isNull()) {
                    std::cout << "ID: ";
                    if (feature->id().isString()) {
                        std::cout << feature->id().asString() << std::endl;
                    } else if (feature->id().isNumber()) {
                        std::cout << feature->id().asNumber() << std::endl;
                    }
                }
                std::cout << "Properties:" << std::endl;
                for (const auto& prop : feature->properties()) {
                    std::cout << prop.first << ": ";
                    if (prop.second.isString()) {
                        std::cout << prop.second.asString() << std::endl;
                    } else if (prop.second.isNumber()) {
                        std::cout << prop.second.asNumber() << std::endl;
                    } else if (prop.second.isBoolean()) {
                        std::cout << (prop.second.asBool() ? "true" : "false") << std::endl;
                    } else if (prop.second.isNull()) {
                        std::cout << "null" << std::endl;
                    } else {
                        std::cout << "[Unsupported property type]" << std::endl;
                    }
                }
                std::cout << "Geometry:" << std::endl;
                if (feature->geometry()) {
                    printGeometry(feature->geometry());
                } else {
                    std::cout << "null" << std::endl;
                }
                break;
            }
            case ObjectType::FeatureCollection: {
                auto featureCollection = std::static_pointer_cast<FeatureCollection>(obj);
                std::cout << "FeatureCollection with " << featureCollection->features().size() << " features:" << std::endl;
                for (const auto& feature : featureCollection->features()) {
                    printGeoJSONObject(feature);
                }
                break;
            }
            default:
                std::cout << "Unsupported GeoJSON object type." << std::endl;
        }
    }

    namespace utils {
        std::vector<std::vector<double>> extractFirstPolygon(
            const std::shared_ptr<geojson::GeoJSONObject>& obj
        ) {
            using ObjectType = geojson::GeoJSONObject::ObjectType;
            using GeometryType = geojson::Geometry::Type;

            std::vector<std::vector<double>> coordinates;


            if (obj->objectType() == ObjectType::FeatureCollection) {
                auto featureCollection = std::static_pointer_cast<geojson::FeatureCollection>(obj);
                if (!featureCollection->features().empty()) {
                    auto feature = featureCollection->features().front();
                    auto geometry = feature->geometry();
                    if (geometry && geometry->type() == GeometryType::Polygon) {
                        auto polygon = std::static_pointer_cast<geojson::Polygon>(geometry);
                        // Get the first ring
                        if (!polygon->rings().empty()) {
                            const auto& ring = polygon->rings().front();
                            for (const auto& coord : ring) {
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
                auto feature = std::static_pointer_cast<geojson::Feature>(obj);
                auto geometry = feature->geometry();
                if (geometry && geometry->type() == GeometryType::Polygon) {
                    auto polygon = std::static_pointer_cast<geojson::Polygon>(geometry);
                    if (!polygon->rings().empty()) {
                        const auto& ring = polygon->rings().front();
                        for (const auto& coord : ring) {
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
                auto geometry = std::static_pointer_cast<geojson::Geometry>(obj);
                if (geometry->type() == GeometryType::Polygon) {
                    auto polygon = std::static_pointer_cast<geojson::Polygon>(geometry);
                    if (!polygon->rings().empty()) {
                        const auto& ring = polygon->rings().front();
                        for (const auto& coord : ring) {
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
    }

} // namespace geojson

#endif // GEOJSON_PARSER_HPP
