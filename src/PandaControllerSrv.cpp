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
            << VERSION_PATCH << "\n"
            << "Build Type: " << BUILD_TYPE << "\n"
            << "Build Timestamp: " << BUILD_TIMESTAMP << "\n"
            << "Compiler: " << COMPILER_ID << " " << COMPILER_VERSION << "\n";
}

bool moveTo(panda_controller::MoveTo::Request& req, panda_controller::MoveTo::Response& res) {
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    const tf2::Quaternion orientation(
            (double) req.orientation_x,
            (double) req.orientation_y,
            (double) req.orientation_z,
            (double) req.orientation_w);

    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = (double) req.position_x;
    grasps[0].grasp_pose.pose.position.y = (double) req.position_y;
    grasps[0].grasp_pose.pose.position.z = (double) req.position_z;

     ROS_INFO("Hyper, Hyper");
    res.was_success = true;
    return true;
}

int main(int argc, char* argv[]) {
    printBuildInfo();
    //    MicroplateTransport trans;
    //    std::cout << trans.MICROPLATE_HEIGHT_IN_M << "\n";

    ros::init(argc, argv, "panda_arm_controller");
    ros::NodeHandle node;

    ros::ServiceServer service = node.advertiseService("move_to", moveTo);
    ROS_INFO("Ready to move.");
    ros::spin();

//    ros::AsyncSpinner spinner(1);
//    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(45.0);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    std::vector<double> jointValues0{
        0.03981577981008856,
        -1.066839368897888,
        -0.1417231063074033,
        -2.295944596229565,
        -0.09524576377316192,
        1.3426527882925567,
        -2.5195802228194144};

    std::vector<double> jointValues1 = {
        0.043933380036991154,
        -1.2048149657667728,
        -0.14132218451204462,
        -2.2845054997673544,
        -0.09504651506741842,
        1.1336760935516015,
        -2.5337736336928276
    };
    group.setJointValueTarget(jointValues0);
    group.move();

    ros::WallDuration(1.0).sleep();

    group.setJointValueTarget(jointValues1);
    group.move();
    for (const auto& val : group.getCurrentJointValues()) {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    group.clearPathConstraints();
    ros::waitForShutdown();

    return 0;
}
