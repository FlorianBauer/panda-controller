#include "Site.h"
#include "ServiceDefs.h"

constexpr double FINGER_LENGTH = 0.20;

Site::Site(const geometry_msgs::Pose& pose) {
    locationPose = pose;
    grasp.grasp_pose.pose = pose;
    grasp.grasp_pose.pose.position.x -= FINGER_LENGTH;

    grasp.grasp_pose.header.frame_id = PANDA_LINK_BASE;

    grasp.pre_grasp_posture.joint_names.resize(2);
    grasp.pre_grasp_posture.joint_names[0] = PANDA_FINGER_1;
    grasp.pre_grasp_posture.joint_names[1] = PANDA_FINGER_1;

    grasp.grasp_posture.joint_names.resize(2);
    grasp.grasp_posture.joint_names[0] = PANDA_FINGER_1;
    grasp.grasp_posture.joint_names[1] = PANDA_FINGER_1;
}

Site::Site(const Site& orig) {
}

Site::~Site() {
}

geometry_msgs::PoseStamped Site::getSitePose() const {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = PANDA_LINK_BASE;
    pose.pose = locationPose;

    return pose;
}

void Site::setApproach(const moveit_msgs::GripperTranslation& approach) {
    grasp.pre_grasp_approach = approach;
    grasp.pre_grasp_approach.direction.header.frame_id = PANDA_LINK_BASE;
}

/**
 * Gets the pick approach.
 * 
 * @return The gripper translation.
 */
moveit_msgs::GripperTranslation Site::getApproach() const {
    moveit_msgs::GripperTranslation approach;
    approach = grasp.pre_grasp_approach;
    return approach;
}

/**
 * Gets the pick retreat.
 * 
 * @return The gripper retreat.
 */
void Site::setRetreat(const moveit_msgs::GripperTranslation& retreat) {
    grasp.post_grasp_retreat = retreat;
    grasp.post_grasp_retreat.direction.header.frame_id = PANDA_LINK_BASE;
    hasRetreat = true;
}

moveit_msgs::GripperTranslation Site::getRetreat() const {
    moveit_msgs::GripperTranslation retreat;
    retreat = grasp.post_grasp_retreat;
    return retreat;
}

moveit_msgs::Grasp& Site::getGrasp() {
    return grasp;
}