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
#include "panda_controller/GetJoints.h"
#include "panda_controller/GetPose.h"
#include "panda_controller/MoveTo.h"
#include "panda_controller/SetJoints.h"
#include "Config.h"
#include "Plate.h"
#include "ServiceDefs.h"
#include "Site.h"

using moveit::planning_interface::MoveItErrorCode;

constexpr double WELLS_LENGTH_IN_M = 0.12776;
constexpr double WELLS_WIDTH_IN_M = 0.08548;
constexpr double WELLS_HEIGHT_IN_M = 0.01435;

moveit::planning_interface::MoveGroupInterface* moveGroupPtr;
moveit::planning_interface::PlanningSceneInterface* planningScenePtr;

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

/**
 * Opens the gripper to the given amount + 1 cm extra space.
 * 
 * @param posture The posture to apply the operation on.
 * @param width The width in m to open the gripper.
 */
void openGripper(trajectory_msgs::JointTrajectory& posture, double width) {
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = width / 2.0 + 0.005;
    posture.points[0].positions[1] = width / 2.0 + 0.005;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

/**
 * Closes the gripper.
 * 
 * @param posture The posture to apply the operation on.
 */
void closedGripper(trajectory_msgs::JointTrajectory& posture) {
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

bool moveToPosition(panda_controller::MoveTo::Request& req, panda_controller::MoveTo::Response& res) {
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

bool pickFromSite(Site& site, Plate& plate) {
    moveit_msgs::Grasp& grasp = site.getGrasp();
    openGripper(grasp.pre_grasp_posture, plate.getSizeY());
    closedGripper(grasp.grasp_posture);
    const MoveItErrorCode err = moveGroupPtr->pick(plate.getPlateId(), grasp);
    if (err != MoveItErrorCode::SUCCESS) {
        return false;
    }
    return true;
}

bool placeToSite(Site& site, Plate& plate) {
    openGripper(site.getGrasp().pre_grasp_posture, plate.getSizeY());
    const MoveItErrorCode err = moveGroupPtr->place(plate.getPlateId(),{site.getPlaceLocation()});
    if (err != MoveItErrorCode::SUCCESS) {
        return false;
    }
    return true;
}

void grabTest() {
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI, 0, -M_PI_4);
    //    orientation.setRPY(M_PI_2, M_PI_4, M_PI_2);

    Plate myPlate("myTestPlate", WELLS_LENGTH_IN_M, WELLS_WIDTH_IN_M, WELLS_HEIGHT_IN_M);

    geometry_msgs::Pose sitePose;
    sitePose.orientation = tf2::toMsg(orientation);
    sitePose.position.x = 0.15 + myPlate.getSizeX() / 2.0;
    sitePose.position.y = 0.50;
    sitePose.position.z = 0.01 + myPlate.getSizeZ() / 2.0;
    Site mySite("grabSite", sitePose);

    moveit_msgs::GripperTranslation approach;
    approach.direction.vector.x = 1.0;
    approach.min_distance = 0.01;
    approach.desired_distance = 0.05;
    mySite.setApproach(approach);

    moveit_msgs::GripperTranslation retreat;
    retreat.direction.vector.z = 1.0;
    retreat.min_distance = 0.01;
    retreat.desired_distance = 0.05;
    mySite.setRetreat(retreat);
    myPlate.putAtSite(mySite);
    // add plate to scene
    planningScenePtr->applyCollisionObject(myPlate.getCollisonObject());

    pickFromSite(mySite, myPlate);
    placeToSite(mySite, myPlate);

    // remove plate from scene
    planningScenePtr->removeCollisionObjects({myPlate.getPlateId()});
}

bool transportPlate(Site& source, Site& destination, Plate& plate) {

    pickFromSite(source, plate);

    const geometry_msgs::PoseStamped curPose = moveGroupPtr->getCurrentPose(PANDA_LINK_HAND);
    // set-up constraint for liquids
    moveit_msgs::OrientationConstraint oriCon;
    oriCon.header.frame_id = PANDA_LINK_BASE;
    oriCon.link_name = PANDA_LINK_HAND;
    oriCon.orientation = curPose.pose.orientation;
    // restrain X and Y axis to max. +-36° tilt(360° / 10)
    oriCon.absolute_x_axis_tolerance = 2.0 * M_PI / 10.0;
    oriCon.absolute_y_axis_tolerance = 2.0 * M_PI / 10.0;
    // Z axis has full freedom
    oriCon.absolute_z_axis_tolerance = 2.0 * M_PI;
    oriCon.weight = 1.0;
    moveit_msgs::Constraints liquidTransportConstraint;
    liquidTransportConstraint.orientation_constraints.push_back(oriCon);

    moveGroupPtr->setPathConstraints(liquidTransportConstraint);

    placeToSite(destination, plate);

    moveGroupPtr->clearPathConstraints();

    return true;
}

void transportTest() {
    tf2::Quaternion orientation;
    //  orientation.setRPY(-M_PI, 0, -M_PI_4);
    orientation.setRPY(M_PI_2, M_PI_4, M_PI_2);

    Plate myPlate("testPlate", WELLS_LENGTH_IN_M, WELLS_WIDTH_IN_M, WELLS_HEIGHT_IN_M);

    geometry_msgs::Pose poseA;
    poseA.orientation = tf2::toMsg(orientation);
    poseA.position.x = 0.15 + myPlate.getSizeX() / 2.0;
    poseA.position.y = 0.50;
    poseA.position.z = 0.01 + myPlate.getSizeZ() / 2.0;
    Site siteA("siteA", poseA);

    geometry_msgs::Pose poseB;
    poseB.orientation = tf2::toMsg(orientation);
    poseB.position.x = 0.15 + myPlate.getSizeX() / 2.0;
    poseB.position.y = -0.50;
    poseB.position.z = 0.01 + myPlate.getSizeZ() / 2.0;
    Site siteB("siteB", poseB);

    moveit_msgs::GripperTranslation approach;
    approach.direction.vector.x = 1.0;
    approach.min_distance = 0.01;
    approach.desired_distance = 0.05;
    siteA.setApproach(approach);
    siteB.setApproach(approach);

    myPlate.putAtSite(siteA);
    // add plate to scene
    planningScenePtr->applyCollisionObject(myPlate.getCollisonObject());

    transportPlate(siteA, siteB, myPlate);

    // remove plate from scene
    planningScenePtr->removeCollisionObjects({myPlate.getPlateId()});
}

/**
 * Get the current pose.
 * 
 * @return `true` on success, otherwise `false`.
 */
bool getPose(panda_controller::GetPose::Request& req, panda_controller::GetPose::Response& res) {
    const geometry_msgs::PoseStamped curPose = moveGroupPtr->getCurrentPose(PANDA_LINK_HAND);
    res.pos_x = curPose.pose.position.x;
    res.pos_y = curPose.pose.position.y;
    res.pos_z = curPose.pose.position.z;
    res.ori_x = curPose.pose.orientation.x;
    res.ori_y = curPose.pose.orientation.y;
    res.ori_z = curPose.pose.orientation.z;
    res.ori_w = curPose.pose.orientation.w;

    //    grabTest();
    transportTest();

    return true;
}

int main(int argc, char* argv[]) {
    printBuildInfo();

    ros::init(argc, argv, PANDA_SERVICE_NAME);
    moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(PANDA_ARM);
    moveGroupPtr->setPlanningTime(15.0);
    planningScenePtr = new moveit::planning_interface::PlanningSceneInterface;

    ros::NodeHandle node;
    const std::vector<ros::ServiceServer> services = {
        node.advertiseService(SRV_GET_JOINTS, getJoints),
        node.advertiseService(SRV_GET_POSE, getPose),
        node.advertiseService(SRV_MOVE_TO, moveToPosition),
        node.advertiseService(SRV_SET_JOINTS, setJoints)
    };

    ROS_INFO("Services are now available.");

    ros::AsyncSpinner spinner(services.size());
    spinner.start();
    ros::waitForShutdown();
    delete moveGroupPtr;
    delete planningScenePtr;

    return 0;
}
