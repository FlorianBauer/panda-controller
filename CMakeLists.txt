cmake_minimum_required(VERSION 3.13)

project("panda_controller")
set(ROBOT_SERVICE "RobotService")
set(PROJECT_TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test")

# Enable tests with -DBUILD_TESTS=ON
option(BUILD_TESTS "Build all unit tests." OFF)
set(CMAKE_BUILD_TYPE "Release")

# Version number
set("${ROBOT_SERVICE}_VERSION_MAJOR" 0)
set("${ROBOT_SERVICE}_VERSION_MINOR" 1)
set("${ROBOT_SERVICE}_VERSION_PATCH" 0)

# Build-timestamp
string(TIMESTAMP BUILD_TIMESTAMP UTC)

find_package(Git)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_SHORT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()


# Set C++17 as standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# General compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Enable multi-threading
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# A header file to pass some of the CMake settings
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Config.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Config.h"
)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/")
include_directories("/opt/ros/${ROS_DISTRO}/include/")

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/json/")

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
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
    roscpp
    rospy
    std_msgs
    message_generation
)

# Generate services in the 'srv' folder
add_service_files(
    FILES
    GetJoints.srv
    GetPose.srv
    MoveTo.srv
    SetJoints.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Build the ROS client node
set(ROBOT_CLIENT "robot_client")
add_executable(${ROBOT_CLIENT} "${CMAKE_CURRENT_SOURCE_DIR}/src/RobotClient.cpp"
                                "${CMAKE_CURRENT_SOURCE_DIR}/src/Plate.cpp"
                                "${CMAKE_CURRENT_SOURCE_DIR}/src/Site.cpp"
                                "${CMAKE_CURRENT_SOURCE_DIR}/src/FileManager.cpp")
add_dependencies(${ROBOT_CLIENT} panda_controller_gencpp)
target_link_libraries(${ROBOT_CLIENT} PRIVATE
    nlohmann_json::nlohmann_json
    ${catkin_LIBRARIES}
)

# Build the ROS service node
add_executable(${ROBOT_SERVICE} "${CMAKE_CURRENT_SOURCE_DIR}/src/RobotService.cpp"
                                "${CMAKE_CURRENT_SOURCE_DIR}/src/Plate.cpp"
                                "${CMAKE_CURRENT_SOURCE_DIR}/src/Site.cpp"
                                "${CMAKE_CURRENT_SOURCE_DIR}/src/FileManager.cpp")
add_dependencies(${ROBOT_SERVICE} panda_controller_gencpp)
target_link_libraries(${ROBOT_SERVICE} PRIVATE
    nlohmann_json::nlohmann_json
    ${catkin_LIBRARIES}
)
