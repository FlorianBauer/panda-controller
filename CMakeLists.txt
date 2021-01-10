cmake_minimum_required(VERSION 3.13)

project("panda_controller")
set(ROBOT_SERVICE "RobotService")
set(PROJECT_TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Set the local gRPC installation directory.
# (see https://gitlab.com/SiLA2/sila_cpp/-/blob/master/BUILDING.md#grpc)
set(LOCAL_INSTALL_DIR "~/.local")
list(APPEND CMAKE_PREFIX_PATH "${LOCAL_INSTALL_DIR}")

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

# Include the JSON library from the git submodule.
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

# Find the sila_cpp library.
# (see https://gitlab.com/SiLA2/sila_cpp/-/blob/master/BUILDING.md#sila_cpp)
if(NOT TARGET sila_cpp)
    find_package(sila_cpp CONFIG REQUIRED)
else()
    message(FATAL_ERROR 
            "Could not find the sila_cpp library. Please install the library first on your system. \
            Instructions: https://gitlab.com/SiLA2/sila_cpp/-/blob/master/BUILDING.md#sila_cpp")
endif()

set(PROTOS
    "${CMAKE_CURRENT_SOURCE_DIR}/meta/SiteManager.proto"
)

protoc_generate_cpp(PROTO_SRCS IMPL_SRCS ${PROTOS})

set(PANDA_CONTROLLER "PandaController")
add_library(${PANDA_CONTROLLER}Features
    ${PROTO_SRCS}
    ${IMPL_SRCS}
)

target_link_libraries(${PANDA_CONTROLLER}Features
    sila2::sila_cpp
)

# The SiLA Server takes the role of the ROS client node.
set(SILA_SERVER "PandaControllerServer")
add_executable(${SILA_SERVER} "${CMAKE_CURRENT_SOURCE_DIR}/${SILA_SERVER}.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Plate.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Site.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/FileManager.cpp"
)

add_dependencies(${SILA_SERVER} panda_controller_gencpp)
target_link_libraries(${SILA_SERVER}
    ${catkin_LIBRARIES}
    nlohmann_json::nlohmann_json
    sila2::sila_cpp
    ${PANDA_CONTROLLER}Features
)

# The server needs the FDL files which are compiled into the binary using the resource file.
target_sources(${SILA_SERVER} PRIVATE "${PANDA_CONTROLLER}.qrc")

# Copy the libs to the directory of the executables so that they can find them.
add_custom_command(TARGET ${SILA_SERVER} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:sila2::sila_cpp>
        $<TARGET_FILE:QtZeroConf>
        $<TARGET_FILE_DIR:${SILA_SERVER}>
)

# Build the ROS service node.
add_executable(${ROBOT_SERVICE} "${CMAKE_CURRENT_SOURCE_DIR}/src/${ROBOT_SERVICE}.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Plate.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Site.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/FileManager.cpp"
)
add_dependencies(${ROBOT_SERVICE} panda_controller_gencpp)
target_link_libraries(${ROBOT_SERVICE} PRIVATE
    nlohmann_json::nlohmann_json
    ${catkin_LIBRARIES}
)
