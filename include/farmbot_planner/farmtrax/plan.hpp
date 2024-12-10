#ifndef PLAN_HPP
#define PLAN_HPP

#include "farmbot_planner/farmtrax/swath.hpp"
#include "mesh.hpp"
#include <boost/graph/graph_traits.hpp>
#include <rclcpp/visibility_control.hpp>
#include <vector>
#include <string>
#include <unordered_set>
#include <stack>
#include <algorithm>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>
#include "spdlog/spdlog.h"
namespace echo = spdlog;

#include "rclcpp/rclcpp.hpp"

namespace farmtrax {
    class Plan {
        private:
            rclcpp::Node::SharedPtr node_;          // ROS 2 node handle

        public:


    };
} // namespace farmtrax

#endif // ROUTE_HPP

// // Iterate over each group based on n
// if (one){
//     std::vector<Swath> temp;
//     for (int i = 0; i < alternate_freq; ++i) {
//         std::vector<Swath> group; // Temporary group to collect elements
//         // Iterate through the vector, stepping by n
//         for (size_t j = i; j < swaths.size(); j += alternate_freq) {
//             group.push_back(swaths[j]);
//         }
//         // If the group index is odd, reverse the group
//         if (i % 2 != 0) {
//             std::reverse(group.begin(), group.end());
//         }
//         // Append the group to the temp vector
//         temp.insert(temp.end(), group.begin(), group.end());
//     }
//     swaths_.push_back(temp);
// } else {
//     for (int i = 0; i < alternate_freq; ++i) {
//         // Iterate through the vector, stepping by n
//         std::vector<Swath> temp;
//         for (size_t j = i; j < swaths.size(); j += alternate_freq) {
//             temp.push_back(swaths[j]);
//         }
//         swaths_.push_back(temp);
//     }
// }
