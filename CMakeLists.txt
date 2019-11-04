cmake_minimum_required(VERSION 2.8.3)
project(obstacle_estimator)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        message_generation
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
 include/${PROJECT_NAME}
)

add_executable(obstacle_filter_node nodes/obstacle_filter_node.cpp src/obstacle_filter.cpp)
target_link_libraries(obstacle_filter_node ${catkin_LIBRARIES})
add_executable(obstacle_prediction_node nodes/obstacle_prediction_node.cpp src/obstacle_prediction.cpp)
target_link_libraries(obstacle_prediction_node ${catkin_LIBRARIES})

## For debugging
#set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
#set (CMAKE_VERBOSE_MAKEFILE ON)
