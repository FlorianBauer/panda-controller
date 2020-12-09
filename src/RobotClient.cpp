/**
 * A ROS client which sends ROS messages to the `robot_service` node.
 */
#include <cstdlib>
#include <ros/ros.h>
#include "RobotClient.h"
#include "ServiceDefs.h"
#include "panda_controller/MoveTo.h"
#include "panda_controller/SetJoints.h"

RobotClient::RobotClient() {
}

RobotClient::RobotClient(const RobotClient& orig) {
}

RobotClient::~RobotClient() {
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, PANDA_CLIENT_NAME);
    ros::NodeHandle node;

    // shaker
    //    pos_x: -0.4211
    //    pos_y: -0.1057
    //    pos_z: 0.1221
    //    rot_p: 0.0083
    //    rot_r: 3.1348
    //    rot_y: -1.4114

    ros::ServiceClient moveClient = node.serviceClient<panda_controller::MoveTo>(SRV_MOVE_TO);
    panda_controller::MoveTo move;
    move.request.pos_x = -0.4211;
    move.request.pos_y = -0.1057;
    move.request.pos_z = 0.1221;
    move.request.rot_r = 0.0083;
    move.request.rot_p = 3.1348;
    move.request.rot_y = -1.4114;

    ROS_INFO("Call move");
    if (moveClient.call(move)) {
        ROS_INFO("Was Success: %d", move.response.was_success);
        ;
    } else {
        ROS_ERROR("Failed to call service MoveTo");
        return 1;
    }

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

    ROS_INFO("Call joints");
    if (jointClient.call(joints)) {
        ROS_INFO("Was Success: %d", joints.response.was_success);
    } else {
        ROS_ERROR("Failed to call service SetJoints");
        return 1;
    }

    return 0;
}