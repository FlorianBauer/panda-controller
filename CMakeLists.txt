cmake_minimum_required(VERSION 3.10)

project("panda-controller")
set(EXECUTABLE_NAME "panda_controller")
set(PROJECT_TEST_DIR "${CMAKE_SOURCE_DIR}/test")

# Enable tests with -DBUILD_TESTS=ON
option(BUILD_TESTS "Build all unit tests." OFF)

# Version number
set("${EXECUTABLE_NAME}_VERSION_MAJOR" 0)
set("${EXECUTABLE_NAME}_VERSION_MINOR" 1)
set("${EXECUTABLE_NAME}_VERSION_PATCH" 0)

# Build-timestamp
string(TIMESTAMP BUILD_TIMESTAMP UTC)

# Set C++11 as standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# General compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Enable multithreading
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


# A header file to pass some of the CMake settings
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Config.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Config.h"
)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/")

# Build Java wrapper
# add_subdirectory("${CMAKE_SOURCE_DIR}/src/wrapper/java/")

# Build utility tools
# add_subdirectory("${CMAKE_SOURCE_DIR}/src/util/")

add_executable(${EXECUTABLE_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/src/PandaController.cpp")
target_sources(${EXECUTABLE_NAME} PUBLIC 
    #"${CMAKE_SOURCE_DIR}/src/Telegram.cpp"
)

target_link_libraries(${EXECUTABLE_NAME}
    Threads::Threads
)

#set(catkin_DIR "/home/bauer/ws_moveit/logs/catkin_tools_prebuild")
#set(catkin_DIR "/opt/ros/melodic/share/catkin/cmake/")
include_directories("/opt/ros/melodic/include/")

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
#    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

#set(THIS_PACKAGE_INCLUDE_DIRS  "doc/interactivity/include")

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
#    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_subdirectory("pick_place/")
