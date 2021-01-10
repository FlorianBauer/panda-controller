/**
 * A ROS client which sends ROS messages to the `robot_service` node.
 */
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nlohmann/json.hpp>
#include "RobotClient.h"
#include "ServiceDefs.h"
#include "panda_controller/GetPose.h"
#include "panda_controller/GetJoints.h"
#include "panda_controller/MoveTo.h"
#include "panda_controller/SetJoints.h"
#include "FileManager.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

RobotClient::RobotClient() {
}

RobotClient::RobotClient(const RobotClient& orig) {
}

RobotClient::~RobotClient() {
}

void loadFiles() {
    const fs::path appDir = FileManager::getAppDir();
    const fs::path sitesDir = appDir / SITES_DIR;
    const fs::path plateTypesDir = appDir / PLATE_TYPES_DIR;
    FileManager::checkAndCreateDir(sitesDir);
    FileManager::checkAndCreateDir(plateTypesDir);

    const std::vector<fs::path> sitesFiles = FileManager::collectJsonFilesFromDir(sitesDir);
    for (const auto& file : sitesFiles) {
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
    }

    const std::vector<fs::path> plateTypesFiles = FileManager::collectJsonFilesFromDir(plateTypesDir);
    for (const auto& file : plateTypesFiles) {
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
        std::cout << "Loaded: " << jsonStruct["type"] << " from " << file << "\n";
    }
}

int main(int argc, char* argv[]) {

    loadFiles();

    ros::init(argc, argv, PANDA_CLIENT_NAME);
    ros::NodeHandle node;

    // shaker
    //    pos_x: -0.4211
    //    pos_y: -0.1057
    //    pos_z: 0.1221
    //    rot_p: 0.0083
    //    rot_r: 3.1348
    //    rot_y: -1.4114

    //    ros::ServiceClient moveClient = node.serviceClient<panda_controller::MoveTo>(SRV_MOVE_TO);
    //    panda_controller::MoveTo move;
    //    move.request.pos_x = -0.4211;
    //    move.request.pos_y = -0.1057;
    //    move.request.pos_z = 0.1221;
    //    
    //    tf2::Quaternion orientation;
    //    //    orientation.setRPY(0.0083, 3.1348, -1.4114);
    //    move.request.ori_x = orientation.getX();
    //    move.request.ori_y = orientation.getY();
    //    move.request.ori_z = orientation.getZ();
    //    move.request.ori_w = orientation.getW();
    //
    //    ROS_INFO("Call MoveTo");
    //    if (moveClient.call(move)) {
    //        ROS_INFO("Was Success: %d", move.response.was_success);
    //        ;
    //    } else {
    //        ROS_ERROR("Failed to call service MoveTo");
    //        return 1;
    //    }

    ros::WallDuration(1.0).sleep();

    ros::ServiceClient jointClient = node.serviceClient<panda_controller::SetJoints>(SRV_SET_JOINTS);
    panda_controller::SetJoints joints;
    joints.request.joints[0] = 0.03981577981008856;
    joints.request.joints[1] = -1.066839368897888;
    joints.request.joints[2] = -0.1417231063074033;
    joints.request.joints[3] = -2.295944596229565;
    joints.request.joints[4] = -0.09524576377316192;
    joints.request.joints[5] = 1.3426527882925567;
    joints.request.joints[6] = -2.5195802228194144;

    ROS_INFO("Call SetJoints");
    if (jointClient.call(joints)) {
        ROS_INFO("Was Success: %d", joints.response.was_success);
    } else {
        ROS_ERROR("Failed to call service SetJoints");
        return 1;
    }

    ros::ServiceClient getJointsClient = node.serviceClient<panda_controller::GetJoints>(SRV_GET_JOINTS);
    panda_controller::GetJoints curJoints;
    ROS_INFO("Call GetJoints");
    if (getJointsClient.call(curJoints)) {
        ROS_INFO("Was Success: [ %f, %f, %f, %f, %f, %f, %f ]",
                curJoints.response.joints[0],
                curJoints.response.joints[1],
                curJoints.response.joints[2],
                curJoints.response.joints[3],
                curJoints.response.joints[4],
                curJoints.response.joints[5],
                curJoints.response.joints[6]);
    } else {
        ROS_ERROR("Failed to call service GetPose");
        return 1;
    }

    ros::ServiceClient getPoseClient = node.serviceClient<panda_controller::GetPose>(SRV_GET_POSE);
    panda_controller::GetPose pose;
    ROS_INFO("Call GetPose");
    if (getPoseClient.call(pose)) {
        ROS_INFO("Was Success: [ %f, %f, %f, %f, %f, %f, %f ]",
                pose.response.pos_x,
                pose.response.pos_y,
                pose.response.pos_x,
                pose.response.ori_x,
                pose.response.ori_y,
                pose.response.ori_z,
                pose.response.ori_w);
    } else {
        ROS_ERROR("Failed to call service GetPose");
        return 1;
    }

    return 0;
}