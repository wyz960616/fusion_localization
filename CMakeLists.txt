cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

set(NAMESPACES_OPEN "fusion_localization")

#add_definitions(-DPRINT_TEST_INFO)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        sensor_msgs
        pcl_ros
        )
set(BACKEND_LIBRARIES "")
set(ALL_BASIC_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/eigen.cmake)
include(cmake/g2o.cmake)
include(cmake/sophus.cmake)
include(cmake/global_definition.cmake)
include(cmake/OpenCV.cmake)
include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB DATA_MGMT_SRCS
        "src/sensors/*.cpp"
        "src/subscribers/*.cpp"
        "src/publishers/*.cpp"
        "src/tools/*.cpp"
        )
file(GLOB_RECURSE MAPPING_SRCS
        "src/mapping/*.cpp"
        )
file(GLOB_RECURSE MODEL_SRCS
        "src/models/*.cpp"
        )

message(STATUS ${MAPPING_SRCS})

add_library(DATA_MGMT_LIB SHARED ${DATA_MGMT_SRCS} src/sensors/point_type.cpp)
target_link_libraries(DATA_MGMT_LIB ${ALL_BASIC_LIBRARIES} ${OpenCV_LIBS})


message(STATUS WORINIDABA)
add_library(MODELS_LIB SHARED ${MODEL_SRCS})
target_link_libraries(MODELS_LIB ${ALL_BASIC_LIBRARIES} DATA_MGMT_LIB)

add_library(MAPPING_LIB SHARED ${MAPPING_SRCS})
target_link_libraries(MAPPING_LIB ${ALL_BASIC_LIBRARIES} ${BACKEND_LIBRARIES} MODELS_LIB DATA_MGMT_LIB)

add_executable(sensor_test src/module_tests/sensors_test.cpp)
target_link_libraries(sensor_test ${catkin_LIBRARIES} DATA_MGMT_LIB)

add_executable(data_pretreat_node "src/ros_nodes/data_pretreat_node.cpp" "src/data_pretreat/data_pretreat.cpp")
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} DATA_MGMT_LIB)

add_executable(front_end_node "src/ros_nodes/front_end_node.cpp")
target_link_libraries(front_end_node ${catkin_LIBRARIES} MAPPING_LIB)