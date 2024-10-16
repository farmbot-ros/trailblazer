#ifndef ROUTE_HPP
#define ROUTE_HPP

#include "field.hpp"
#include "swath.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <boost/geometry/algorithms/distance.hpp> // For calculating distance

// Include Boost libraries for graph and R-tree
#include <boost/geometry/index/rtree.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>

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

    class Route {
        private:
            Field inner_field_;
            Field outer_field_;
            Swaths swaths_;

        public:
            Route() = default;

        private:
            
    };

} // namespace farmtrax

#endif // ROUTE_HPP
