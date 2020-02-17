/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

constexpr double WELLS_LENGTH_IN_M = 0.12776;
constexpr double WELLS_WIDTH_IN_M = 0.08548;
constexpr double WELLS_HEIGHT_IN_M = 0.01435;

void openGripper(trajectory_msgs::JointTrajectory& posture) {
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = WELLS_WIDTH_IN_M / 2.0 + 0.01;
    posture.points[0].positions[1] = WELLS_WIDTH_IN_M / 2.0 + 0.01;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture) {
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group) {
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
    // of the cube). |br|
    // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
    // extra padding)
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI, 0, -M_PI_4);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.15 + WELLS_LENGTH_IN_M / 2.0;
    grasps[0].grasp_pose.pose.position.y = 0.50;
    grasps[0].grasp_pose.pose.position.z = 0.25;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);

    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table2.
    move_group.setSupportSurfaceName("table2");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group) {
    // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    // location in
    // verbose mode." This is a known issue and we are working on fixing it. |br|
    // Create a vector of placings to be attempted, currently only creating single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI_2);
    //  orientation.setRPY(0, M_PI / 2, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* While placing it is the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = -0.4;
    place_location[0].place_pose.pose.position.y = 0.2;
    place_location[0].place_pose.pose.position.z = 0.025;

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table1.
    group.setSupportSurfaceName("table1");
    // Call place to place the object using the place locations given.
    group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(6);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "panda_link0";
    // Define the primitive and its dimensions.
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 2.00; // length
    collision_objects[0].primitives[0].dimensions[1] = 0.70; // width
    collision_objects[0].primitives[0].dimensions[2] = 0.05; // height
    // Define the pose of the table.
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.00;
    collision_objects[0].primitive_poses[0].position.y = 0.00;
    collision_objects[0].primitive_poses[0].position.z = -0.025;
    collision_objects[0].operation = collision_objects[0].ADD;

    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "panda_link0";
    // Define the primitive and its dimensions.
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.90;
    collision_objects[1].primitives[0].dimensions[1] = 1.20;
    collision_objects[1].primitives[0].dimensions[2] = 0.05;
    // Define the pose of the table.
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.55;
    collision_objects[1].primitive_poses[0].position.y = 0.95;
    collision_objects[1].primitive_poses[0].position.z = -0.025;
    collision_objects[1].operation = collision_objects[1].ADD;

    // Add the centrifuge.
    constexpr double CENTRIFUGE_DEPTH = 0.605;
    constexpr double CENTRIFUGE_WIDTH = 0.623;
    constexpr double CENTRIFUGE_HEIGHT = 0.36;
    collision_objects[2].id = "centrifuge";
    collision_objects[2].header.frame_id = "panda_link0";
    // Define the primitive and its dimensions.
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = CENTRIFUGE_DEPTH;
    collision_objects[2].primitives[0].dimensions[1] = CENTRIFUGE_WIDTH;
    collision_objects[2].primitives[0].dimensions[2] = CENTRIFUGE_HEIGHT;
    // Define the pose of the centrifuge.
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = CENTRIFUGE_DEPTH / 2.0 + 0.06 + 0.18;
    collision_objects[2].primitive_poses[0].position.y = 0.00;
    collision_objects[2].primitive_poses[0].position.z = CENTRIFUGE_HEIGHT / 2.0;
    collision_objects[2].operation = collision_objects[2].ADD;

    // Add the Biomek base.
    collision_objects[3].id = "biomek0";
    collision_objects[3].header.frame_id = "panda_link0";
    // Define the primitive and its dimensions.
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.60;
    collision_objects[3].primitives[0].dimensions[1] = 1.00;
    collision_objects[3].primitives[0].dimensions[2] = 0.20;
    // Define the pose of the centrifuge.
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.30 + 0.30;
    collision_objects[3].primitive_poses[0].position.y = 0.50 + 0.40;
    collision_objects[3].primitive_poses[0].position.z = 0.10;
    collision_objects[3].operation = collision_objects[3].ADD;

    collision_objects[4].id = "biomek1";
    collision_objects[4].header.frame_id = "panda_link0";
    // Define the primitive and its dimensions.
    collision_objects[4].primitives.resize(1);
    collision_objects[4].primitives[0].type = collision_objects[4].primitives[0].BOX;
    collision_objects[4].primitives[0].dimensions.resize(3);
    collision_objects[4].primitives[0].dimensions[0] = 0.60;
    collision_objects[4].primitives[0].dimensions[1] = 0.05;
    collision_objects[4].primitives[0].dimensions[2] = 1.20;
    // Define the pose of the centrifuge.
    collision_objects[4].primitive_poses.resize(1);
    collision_objects[4].primitive_poses[0].position.x = 0.30 + 0.30;
    collision_objects[4].primitive_poses[0].position.y = 0.025 + 0.35;
    collision_objects[4].primitive_poses[0].position.z = 0.60;
    collision_objects[4].operation = collision_objects[4].ADD;

    // Define the object that we will be manipulating
    collision_objects[5].header.frame_id = "panda_link0";
    collision_objects[5].id = "object";
    // Define the primitive and its dimensions.
    collision_objects[5].primitives.resize(1);
    collision_objects[5].primitives[0].type = collision_objects[5].primitives[0].BOX;
    collision_objects[5].primitives[0].dimensions.resize(3);
    collision_objects[5].primitives[0].dimensions[0] = WELLS_LENGTH_IN_M;
    collision_objects[5].primitives[0].dimensions[1] = WELLS_WIDTH_IN_M;
    collision_objects[5].primitives[0].dimensions[2] = WELLS_HEIGHT_IN_M;
    // Define the pose of the object.
    collision_objects[5].primitive_poses.resize(1);
    collision_objects[5].primitive_poses[0].position.x = 0.15 + WELLS_LENGTH_IN_M / 2.0;
    collision_objects[5].primitive_poses[0].position.y = 0.50;
    collision_objects[5].primitive_poses[0].position.z = 0.01 + WELLS_HEIGHT_IN_M / 2.0;
    collision_objects[5].operation = collision_objects[5].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(45.0);

    // setup constrains
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "panda_link7";
    ocm.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI, 0, -M_PI_4);
    ocm.orientation = tf2::toMsg(orientation);
    ocm.absolute_x_axis_tolerance = M_PI / 10.0;
    ocm.absolute_y_axis_tolerance = M_PI / 10.0;
    ocm.absolute_z_axis_tolerance = 2 * M_PI;
    ocm.weight = 1.0;
    moveit_msgs::Constraints grasp_constrains;
    grasp_constrains.orientation_constraints.push_back(ocm);

    addCollisionObjects(planning_scene_interface);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    pick(group);
    group.setPathConstraints(grasp_constrains);
    ros::WallDuration(1.0).sleep();
    place(group);
    group.clearPathConstraints();
    ros::waitForShutdown();
    return 0;
}
