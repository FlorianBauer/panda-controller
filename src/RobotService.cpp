/** 
 * The ROS service which receives the ROS messages from the `robot_client` and executes the actual
 * movements. Only low-level commands are done here. High-level tasks (e.g.  collision detection) 
 * are in the responsibility of the client.
 */
#include <iostream>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Config.h"
#include "ServiceDefs.h"
#include "panda_controller/GetJoints.h"
#include "panda_controller/GetPose.h"
#include "panda_controller/MoveTo.h"
#include "panda_controller/SetJoints.h"

moveit::planning_interface::MoveGroupInterface* moveGroupPtr;

/**
 * Prints some information about the current build.
 */
static void printBuildInfo() {
    std::cout << EXECUTABLE_NAME << "\n"
            << "Version: "
            << VERSION_MAJOR << "."
            << VERSION_MINOR << "."
            << VERSION_PATCH << "-"
            << GIT_COMMIT_SHORT << "\n"
            << "Build Type: " << BUILD_TYPE << "\n"
            << "Build Timestamp: " << BUILD_TIMESTAMP << "\n"
            << "Compiler: " << COMPILER_ID << " " << COMPILER_VERSION << "\n";
}

bool moveTo(panda_controller::MoveTo::Request& req, panda_controller::MoveTo::Response& res) {
    geometry_msgs::Pose targetPose;
    targetPose.position.x = req.pos_x;
    targetPose.position.y = req.pos_y;
    targetPose.position.z = req.pos_z;
    targetPose.orientation.x = req.ori_x;
    targetPose.orientation.y = req.ori_y;
    targetPose.orientation.z = req.ori_z;
    targetPose.orientation.w = req.ori_w;
    moveGroupPtr->setPoseTarget(targetPose);
    moveGroupPtr->move();
    ROS_INFO("moveTo success");
    res.was_success = true;
    return true;
}

/**
 * Sets the angle of each joint an therefore moves the arm to a certain pose. This is effectively 
 * equivalent to `moveTo` but without path planning or collision detection.
 * 
 * @param req The request containing a array with all joint values.
 * @param res The response containing a bool signaling the success of the operation.
 * @return `true` on success, otherwise `false`.
 */
bool setJoints(panda_controller::SetJoints::Request& req, panda_controller::SetJoints::Response& res) {
    const std::vector<double> jointValues = {
        req.joints[0],
        req.joints[1],
        req.joints[2],
        req.joints[3],
        req.joints[4],
        req.joints[5],
        req.joints[6]
    };
    moveGroupPtr->setJointValueTarget(jointValues);
    moveGroupPtr->move();
    ROS_INFO("setJoints success");
    res.was_success = true;
    return true;
}

/**
 * Gets the current joint values.
 * 
 * @return `true` on success, otherwise `false`.
 */
bool getJoints(panda_controller::GetJoints::Request& req, panda_controller::GetJoints::Response& res) {
    const std::vector<double> curJoints = moveGroupPtr->getCurrentJointValues();
    res.joints[0] = curJoints[0];
    res.joints[1] = curJoints[0];
    res.joints[2] = curJoints[0];
    res.joints[3] = curJoints[0];
    res.joints[4] = curJoints[0];
    res.joints[5] = curJoints[0];
    res.joints[6] = curJoints[0];
    return true;
}

/**
 * Get the current pose.
 * 
 * @return `true` on success, otherwise `false`.
 */
bool getPose(panda_controller::GetPose::Request& req, panda_controller::GetPose::Response& res) {
    const geometry_msgs::PoseStamped curPose = moveGroupPtr->getCurrentPose(PANDA_LINK_EEF);
    res.pos_x = curPose.pose.position.x;
    res.pos_y = curPose.pose.position.y;
    res.pos_z = curPose.pose.position.z;
    res.ori_x = curPose.pose.orientation.x;
    res.ori_y = curPose.pose.orientation.y;
    res.ori_z = curPose.pose.orientation.z;
    res.ori_w = curPose.pose.orientation.w;
    return true;
}

int main(int argc, char* argv[]) {
    printBuildInfo();

    ros::init(argc, argv, PANDA_SERVICE_NAME);
    moveit::planning_interface::PlanningSceneInterface planningScene;
    moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(PANDA_ARM);
    moveGroupPtr->setPlanningTime(45.0);

    ros::NodeHandle node;
    const std::vector<ros::ServiceServer> services = {
        node.advertiseService(SRV_GET_JOINTS, getJoints),
        node.advertiseService(SRV_GET_POSE, getPose),
        node.advertiseService(SRV_MOVE_TO, moveTo),
        node.advertiseService(SRV_SET_JOINTS, setJoints)
    };

    ROS_INFO("Services are now available.");

    ros::AsyncSpinner spinner(services.size());
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
