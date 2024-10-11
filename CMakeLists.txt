cmake_minimum_required(VERSION 3.15)
project(farmbot_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)
cmake_policy(SET CMP0074 NEW)

set(executables
    ab_planner
)

set (utils
    fieldgen
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(farmbot_interfaces REQUIRED)

set(dependencies_ros
    rclcpp 
    std_msgs
    std_srvs
    nav_msgs
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    message_filters
    pcl_ros
    pcl_conversions
    farmbot_interfaces
)

option(BUILD_PYTHON "Build Python SWIG module" OFF)
FetchContent_Declare(
    Fields2Cover
    GIT_REPOSITORY https://github.com/Fields2Cover/Fields2Cover.git
    GIT_TAG        main 
)
FetchContent_MakeAvailable(Fields2Cover)
# set_property(DIRECTORY ${fields2cover_SOURCE_DIR} APPEND PROPERTY COMPILE_OPTIONS -Wno-undef)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Eigen_LIBRARY_DIRS})
add_definitions(${Eigen_DEFINITIONS})
find_package(fmt REQUIRED)
find_package(PCL REQUIRED)
find_package(GDAL 3.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# FindGeographicLib.cmake is not installed in the default location, so we need to specify the path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
find_package(nlohmann_json REQUIRED)

set(dependencies_sys
    fmt::fmt
    ${PCL_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${GDAL_LIBRARIES}
    Fields2Cover
    nlohmann_json::nlohmann_json
)

foreach(util ${utils})
    add_library(${util} src/utils/${util}.cpp)
    ament_target_dependencies(${util} ${dependencies_ros})
    target_link_libraries(${util} ${dependencies_sys})
endforeach()

foreach(executable ${executables})
    add_executable(${executable} src/${executable}.cpp)
    ament_target_dependencies(${executable} ${dependencies_ros})
    target_link_libraries(${executable} ${dependencies_sys})
    foreach(util ${utils})
        target_link_libraries(${executable} ${util})
    endforeach()
endforeach()

install(TARGETS
    ${executables}
    DESTINATION lib/${PROJECT_NAME}
)

include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${nlohmann_json_INCLUDE_DIRS}
)


ament_package()
