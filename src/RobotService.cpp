/** 
 * The ROS service node which receives the ROS messages from the client (aka 
 * `PandaControllerServer`) and executes the actual movements. Only low-level commands should be 
 * handled here. High-level tasks are in the responsibility of the client.
 */
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <filesystem>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nlohmann/json.hpp>
#include "panda_controller/FollowFrames.h"
#include "panda_controller/FollowPath.h"
#include "panda_controller/GetCurrentFrame.h"
#include "panda_controller/GetCurrentPose.h"
#include "panda_controller/MoveToPose.h"
#include "panda_controller/SetToFrame.h"
#include "Config.h"
#include "Plate.h"
#include "ServiceDefs.h"
#include "Site.h"
#include "FileManager.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
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

void loadCollisionObjectsIntoScene() {
    const fs::path collObjDir = FileManager::getAppDir() / "collision_objects";
    const std::vector<fs::path> collFiles = FileManager::collectJsonFilesFromDir(collObjDir);
    if (collFiles.empty()) {
        return;
    }

    std::vector<moveit_msgs::CollisionObject> collObjs;
    collObjs.resize(collFiles.size() + 2);

    for (const auto& file : collFiles) {
        // read a JSON file
        std::ifstream jsonStream(file);
        json jsonStruct;
        try {
            jsonStream >> jsonStruct;
        } catch (const std::exception& ex) {
            std::cerr << "Could not load: " << file << "\n" << ex.what();
            jsonStream.close();
            continue;
        }
        jsonStream.close();
        std::cout << "Loaded: " << jsonStruct["id"] << " from " << file << "\n";

        moveit_msgs::CollisionObject collBox;
        collBox.id = jsonStruct["id"].get<std::string>();
        collBox.header.frame_id = PANDA_LINK_BASE;
        collBox.primitives.resize(1);
        collBox.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        collBox.primitive_poses.resize(1);
        collBox.primitive_poses[0].position.x = jsonStruct["posX"];
        collBox.primitive_poses[0].position.y = jsonStruct["posY"];
        collBox.primitive_poses[0].position.z = jsonStruct["posZ"];

        tf2::Quaternion orientation;
        orientation.setRPY(jsonStruct["rotR"], jsonStruct["rotP"], jsonStruct["rotY"]);
        collBox.primitive_poses[0].orientation = tf2::toMsg(orientation);

        collBox.primitives[0].dimensions.resize(3);
        collBox.primitives[0].dimensions[0] = jsonStruct["dimX"]; // length;
        collBox.primitives[0].dimensions[1] = jsonStruct["dimY"]; // width
        collBox.primitives[0].dimensions[2] = jsonStruct["dimZ"]; // hight
        collBox.operation = moveit_msgs::CollisionObject::ADD;
        collObjs.push_back(collBox);
    }
    planningScenePtr->applyCollisionObjects(collObjs);
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

bool moveToPose(panda_controller::MoveToPose::Request& req, panda_controller::MoveToPose::Response& res) {
    geometry_msgs::Pose targetPose;
    targetPose.position.x = req.pos_x;
    targetPose.position.y = req.pos_y;
    targetPose.position.z = req.pos_z;
    targetPose.orientation.x = req.ori_x;
    targetPose.orientation.y = req.ori_y;
    targetPose.orientation.z = req.ori_z;
    targetPose.orientation.w = req.ori_w;
    moveGroupPtr->setPoseTarget(targetPose);
    const MoveItErrorCode err = moveGroupPtr->move();
    return (err == MoveItErrorCode::SUCCESS);
}

/**
 * Sets the angle of each joint an therefore moves the arm to a certain pose. This is effectively 
 * equivalent to `moveTo` but without path planning or collision detection.
 * 
 * @param req The request containing a array with all joint values.
 * @param res The empty response.
 * @return `true` on success, otherwise `false`.
 */
bool setToFrame(panda_controller::SetToFrame::Request& req, panda_controller::SetToFrame::Response& res) {
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
    const MoveItErrorCode err = moveGroupPtr->move();
    return (err == MoveItErrorCode::SUCCESS);
}

/**
 * Gets the current joint values.
 * 
 * @return `true` on success, otherwise `false`.
 */
bool getCurrentFrame(panda_controller::GetCurrentFrame::Request& req, panda_controller::GetCurrentFrame::Response& res) {
    const std::vector<double> curJoints = moveGroupPtr->getCurrentJointValues();
    res.joints[0] = curJoints[0];
    res.joints[1] = curJoints[1];
    res.joints[2] = curJoints[2];
    res.joints[3] = curJoints[3];
    res.joints[4] = curJoints[4];
    res.joints[5] = curJoints[5];
    res.joints[6] = curJoints[6];
    return true;
}

bool pickFromSite(Site& site, Plate& plate) {
    moveit_msgs::Grasp& grasp = site.getGrasp();
    openGripper(grasp.pre_grasp_posture, plate.getDimY());
    closedGripper(grasp.grasp_posture);
    const MoveItErrorCode err = moveGroupPtr->pick(plate.getId(), grasp);
    if (err != MoveItErrorCode::SUCCESS) {
        return false;
    }
    return true;
}

bool placeToSite(Site& site, Plate& plate) {
    openGripper(site.getGrasp().pre_grasp_posture, plate.getDimY());
    const MoveItErrorCode err = moveGroupPtr->place(plate.getId(),{site.getPlaceLocation()});
    return (err == MoveItErrorCode::SUCCESS);
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

/**
 * Get the current pose.
 * 
 * @return `true` on success, otherwise `false`.
 */
bool getCurrentPose(panda_controller::GetCurrentPose::Request& req, panda_controller::GetCurrentPose::Response& res) {
    const geometry_msgs::PoseStamped curPose = moveGroupPtr->getCurrentPose(PANDA_LINK_HAND);
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
    moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(PANDA_ARM);
    moveGroupPtr->setPlanningTime(15.0);
    planningScenePtr = new moveit::planning_interface::PlanningSceneInterface;

    loadCollisionObjectsIntoScene();

    ros::NodeHandle node;
    const std::vector<ros::ServiceServer> services = {
        node.advertiseService(SRV_GET_CURRENT_FRAME, getCurrentFrame),
        node.advertiseService(SRV_GET_CURRENT_POSE, getCurrentPose),
        node.advertiseService(SRV_MOVE_TO_POSE, moveToPose),
        node.advertiseService(SRV_SET_TO_FRAME, setToFrame)
    };

    ROS_INFO("Services are now available.");

    ros::AsyncSpinner spinner(services.size());
    spinner.start();
    ros::waitForShutdown();
    delete moveGroupPtr;
    delete planningScenePtr;

    return 0;
}
