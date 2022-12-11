cmake_minimum_required(VERSION 3.10)

project(se3ompl)

set(CMAKE_CXX_FLAGS "-Wall -std=c++11")

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -std=c++11")

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    tf
    tf2_msgs
    tf2_ros
    tf2_geometry_msgs
    roscpp
    rviz
)

catkin_package(
  LIBRARIES
    rviz_tf_marker
  CATKIN_DEPENDS 
    geometry_msgs
    tf
    tf2_msgs
    tf2_ros
    roscpp
    rviz
)

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_FILES})
add_definitions(${OMPL_DEFINITIONS})


find_package(Boost COMPONENTS system REQUIRED)

add_executable(OMPL_SE3 src/SimpleMotionPlanning.cpp)
target_link_libraries(OMPL_SE3 ${OMPL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(DynamicPlanning src/WindowedPlanning.cpp)
target_link_libraries(DynamicPlanning ${OMPL_LIBRARIES} ${catkin_LIBRARIES})

