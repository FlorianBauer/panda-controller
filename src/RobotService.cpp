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

int main(int argc, char* argv[]) {
    printBuildInfo();

    ros::init(argc, argv, PANDA_SERVICE_NAME);
    moveit::planning_interface::PlanningSceneInterface planningScene;
    moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(PANDA_ARM);
    moveGroupPtr->setPlanningTime(45.0);

    ros::NodeHandle node;
    std::vector<ros::ServiceServer> services = {
        node.advertiseService(SRV_MOVE_TO, moveTo),
        node.advertiseService(SRV_SET_JOINTS, setJoints)
    };

    ROS_INFO("Ready to move.");

    ros::AsyncSpinner spinner(services.size());
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
