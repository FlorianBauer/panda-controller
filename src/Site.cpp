/**
 * A "Site" describes a place or device relative located to the robot to pick things up from or
 * place to. Therefore, every site needs to define a pose from which the robot can access this site.
 */
#include "Site.h"
#include "ServiceDefs.h"

/// The length of the robot finger in m.
constexpr double FINGER_LENGTH = 0.20;
/// Finger position relative to end-effector.
static const tf2::Vector3 FINGER_REL_POS(0.0, 0.0, -FINGER_LENGTH);

Site::Site(const std::string& identifier, const geometry_msgs::Pose& pose) {
    siteId = identifier;
    grasp.grasp_pose.header.frame_id = PANDA_LINK_BASE;
    locationPose = pose;
    grasp.grasp_pose.pose = pose;

    // Transform position a bit back to grab target with the actual fingers.
    tf2::Transform trans;
    tf2::fromMsg(pose, trans);
    tf2::Vector3 transFinger = trans * FINGER_REL_POS;
    grasp.grasp_pose.pose.position.x = transFinger.getX();
    grasp.grasp_pose.pose.position.y = transFinger.getY();
    grasp.grasp_pose.pose.position.z = transFinger.getZ();

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
    if (hasRetreat) {
        retreat = grasp.post_grasp_retreat;
    } else {
        // if no retreat was set, use the reversed approach
        retreat = grasp.pre_grasp_approach;
        retreat.direction.vector.x *= -1.0;
        retreat.direction.vector.y *= -1.0;
        retreat.direction.vector.z *= -1.0;
    }
    return retreat;
}

moveit_msgs::Grasp& Site::getGrasp() {
    return grasp;
}

moveit_msgs::PlaceLocation Site::getPlaceLocation() const {
    geometry_msgs::PoseStamped sitePose = getSitePose();
    tf2::Quaternion orientation;
    sitePose.pose.orientation = tf2::toMsg(orientation);

    moveit_msgs::PlaceLocation pl;
    pl.place_pose = sitePose;

    // The place approach is the reversed pick retreat.
    moveit_msgs::GripperTranslation approach = getRetreat();
    approach.direction.vector.x *= -1.0;
    approach.direction.vector.y *= -1.0;
    approach.direction.vector.z *= -1.0;
    pl.pre_place_approach = approach;

    // The place retreat is the reversed pick approach.
    moveit_msgs::GripperTranslation retreat = getApproach();
    retreat.direction.vector.x *= -1.0;
    retreat.direction.vector.y *= -1.0;
    retreat.direction.vector.z *= -1.0;
    pl.post_place_retreat = retreat;

    return pl;
}
