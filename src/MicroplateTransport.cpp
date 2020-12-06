#include "MicroplateTransport.h"

#include <fstream>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <nlohmann/json.hpp>

MicroplateTransport::MicroplateTransport() {
}

MicroplateTransport::MicroplateTransport(const MicroplateTransport& orig) {
}

MicroplateTransport::~MicroplateTransport() {
}

void MicroplateTransport::openGripper(trajectory_msgs::JointTrajectory& posture) {
    // Add both finger joints of panda robot.
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    // Set them as open, wide enough for the object to fit.
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = MICROPLATE_WIDTH_IN_M / 2.0 + 0.01;
    posture.points[0].positions[1] = MICROPLATE_WIDTH_IN_M / 2.0 + 0.01;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void MicroplateTransport::closeGripper(trajectory_msgs::JointTrajectory& posture) {
    // Add both finger joints of panda robot.
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    // Set them as closed.
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void MicroplateTransport::run() {
    // TODO: implement
}

void MicroplateTransport::loadFrankaEmikaTaskFile() {
//    using json = nlohmann::json;
//
//    std::ifstream taskFile("file.json");
//    json jsonStruct;
//    taskFile >> jsonStruct;
    // TODO: implement
}