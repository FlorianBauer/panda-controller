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
#include "MicroplateTransport.h"
#include "panda_controller/MoveTo.h"

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
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    grasps[0].grasp_pose.pose.position.x = req.pos_x;
    grasps[0].grasp_pose.pose.position.y = req.pos_y;
    grasps[0].grasp_pose.pose.position.z = req.pos_z;
    tf2::Quaternion orientation;
    orientation.setRPY(req.rot_r, req.rot_p, req.rot_y);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    ROS_INFO("Hyper, Hyper");
    res.was_success = true;
    return true;
}

int main(int argc, char* argv[]) {
    printBuildInfo();

    ros::init(argc, argv, "panda_controller_service");
    ros::NodeHandle node;
    ros::ServiceServer service = node.advertiseService("move_to", moveTo);
    ROS_INFO("Ready to move.");
    ros::spin();

    return 0;
}
