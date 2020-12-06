/**
 * A ROS client which sends ROS messages to the `robot_service` node.
 */
#include <cstdlib>
#include <ros/ros.h>
#include "RobotClient.h"
#include "panda_controller/MoveTo.h"



RobotClient::RobotClient() {
}

RobotClient::RobotClient(const RobotClient& orig) {
}

RobotClient::~RobotClient() {
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "panda_controller_client");

    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<panda_controller::MoveTo>("move_to");

    panda_controller::MoveTo srv;
    srv.request.position_x = 5.0;
    srv.request.position_y = 5.0;
    srv.request.position_z = 5.0;
    srv.request.orientation_x = 5.0;
    srv.request.orientation_y = 5.0;
    srv.request.orientation_z = 5.0;
    srv.request.orientation_w = 5.0;

    if (client.call(srv)) {
        ROS_INFO("Was Success: %d", srv.response.was_success);
    } else {
        ROS_ERROR("Failed to call service MoveTo");
        return 1;
    }

    return 0;
}