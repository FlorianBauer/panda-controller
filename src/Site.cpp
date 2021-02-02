/**
 * A "Site" describes a place or device relative located to the robot to pick things up from or
 * put to. Therefore, every Site needs to define a pose from which the arm can access this location.
 */
#include "Site.h"

#include "ServiceDefs.h"

using json = nlohmann::json;

/// The length of the robot finger in m.
static constexpr double FINGER_LENGTH = 0.20;
/// Finger position relative to end-effector.
static const tf2::Vector3 FINGER_REL_POS(0.0, 0.0, -FINGER_LENGTH);

Site::Site(const std::string& identifier) {
    id = identifier;
    grasp.grasp_pose.header.frame_id = PANDA_LINK_BASE;
    grasp.pre_grasp_posture.joint_names.resize(2);
    grasp.pre_grasp_posture.joint_names[0] = PANDA_FINGER_1;
    grasp.pre_grasp_posture.joint_names[1] = PANDA_FINGER_2;

    grasp.grasp_posture.joint_names.resize(2);
    grasp.grasp_posture.joint_names[0] = PANDA_FINGER_1;
    grasp.grasp_posture.joint_names[1] = PANDA_FINGER_2;
}

Site::Site(const json& jsonStruct) {
    id = jsonStruct[SITE_ID].get<std::string>();
    const json& jsonPose = jsonStruct[POSE];
    locationPose.position.x = jsonPose[POS_X].get<double>();
    locationPose.position.y = jsonPose[POS_Y].get<double>();
    locationPose.position.z = jsonPose[POS_Z].get<double>();
    locationPose.orientation.x = jsonPose[ORI_X].get<double>();
    locationPose.orientation.y = jsonPose[ORI_Y].get<double>();
    locationPose.orientation.z = jsonPose[ORI_Z].get<double>();
    locationPose.orientation.w = jsonPose[ORI_W].get<double>();

    if (jsonStruct.find(APPROACH) != jsonStruct.cend()) {
        const json& jsonApproach = jsonStruct[APPROACH];
        grasp.pre_grasp_approach.direction.header.frame_id = PANDA_LINK_BASE;
        grasp.pre_grasp_approach.desired_distance = jsonApproach[DESIRED_DIST].get<double>();
        grasp.pre_grasp_approach.min_distance = jsonApproach[MIN_DIST].get<double>();
        grasp.pre_grasp_approach.direction.vector.x = jsonApproach[DIR_X].get<double>();
        grasp.pre_grasp_approach.direction.vector.y = jsonApproach[DIR_Y].get<double>();
        grasp.pre_grasp_approach.direction.vector.z = jsonApproach[DIR_Z].get<double>();
        hasApproach = true;
    }

    if (jsonStruct.find(RETREAT) != jsonStruct.cend()) {
        const json& jsonRetreat = jsonStruct[RETREAT];
        grasp.post_place_retreat.direction.header.frame_id = PANDA_LINK_BASE;
        grasp.post_place_retreat.desired_distance = jsonRetreat[DESIRED_DIST].get<double>();
        grasp.post_place_retreat.min_distance = jsonRetreat[MIN_DIST].get<double>();
        grasp.post_place_retreat.direction.vector.x = jsonRetreat[DIR_X].get<double>();
        grasp.post_place_retreat.direction.vector.y = jsonRetreat[DIR_Y].get<double>();
        grasp.post_place_retreat.direction.vector.z = jsonRetreat[DIR_Z].get<double>();
        hasRetreat = true;
    }

    grasp.grasp_pose.header.frame_id = PANDA_LINK_BASE;

    // Transform position a bit back to grab target with the actual fingers.
    tf2::Transform trans;
    tf2::fromMsg(locationPose, trans);
    tf2::Vector3 transFinger = trans * FINGER_REL_POS;
    grasp.grasp_pose.pose = locationPose;
    grasp.grasp_pose.pose.position.x = transFinger.getX();
    grasp.grasp_pose.pose.position.y = transFinger.getY();
    grasp.grasp_pose.pose.position.z = transFinger.getZ();

    grasp.pre_grasp_posture.joint_names.resize(2);
    grasp.pre_grasp_posture.joint_names[0] = PANDA_FINGER_1;
    grasp.pre_grasp_posture.joint_names[1] = PANDA_FINGER_2;

    grasp.grasp_posture.joint_names.resize(2);
    grasp.grasp_posture.joint_names[0] = PANDA_FINGER_1;
    grasp.grasp_posture.joint_names[1] = PANDA_FINGER_2;
}

/**
 * Get the identifier of the site.
 * 
 * @return The ID string.
 */
const std::string& Site::getId() const {
    return id;
}

void Site::setPose(const geometry_msgs::Pose& pose) {
    locationPose = pose;

    // Transform position a bit back to grab target with the actual fingers.
    tf2::Transform trans;
    tf2::fromMsg(locationPose, trans);
    tf2::Vector3 transFinger = trans * FINGER_REL_POS;

    grasp.grasp_pose.pose = locationPose;
    grasp.grasp_pose.pose.position.x = transFinger.getX();
    grasp.grasp_pose.pose.position.y = transFinger.getY();
    grasp.grasp_pose.pose.position.z = transFinger.getZ();
}

geometry_msgs::PoseStamped Site::getPose() const {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = PANDA_LINK_BASE;
    pose.pose = locationPose;
    return pose;
}

void Site::setApproach(const moveit_msgs::GripperTranslation& approach) {
    grasp.pre_grasp_approach = approach;
    grasp.pre_grasp_approach.direction.header.frame_id = PANDA_LINK_BASE;
    hasApproach = true;
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

moveit_msgs::Grasp Site::getGrasp() const {
    return moveit_msgs::Grasp(grasp);
}

moveit_msgs::PlaceLocation Site::getPlaceLocation() const {
    geometry_msgs::PoseStamped sitePose = getPose();
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

json Site::toJson() const {
    json jsonStruct;
    jsonStruct[SITE_ID] = id;
    jsonStruct[POSE] = {
        {POS_X, locationPose.position.x},
        {POS_Y, locationPose.position.y},
        {POS_Z, locationPose.position.z},
        {ORI_X, locationPose.orientation.x},
        {ORI_Y, locationPose.orientation.y},
        {ORI_Z, locationPose.orientation.z},
        {ORI_W, locationPose.orientation.w},
    };

    if (hasApproach) {
        jsonStruct[APPROACH] = {
            {DESIRED_DIST, grasp.pre_grasp_approach.desired_distance},
            {MIN_DIST, grasp.pre_grasp_approach.min_distance},
            {DIR_X, grasp.pre_grasp_approach.direction.vector.x},
            {DIR_Y, grasp.pre_grasp_approach.direction.vector.y},
            {DIR_Z, grasp.pre_grasp_approach.direction.vector.z},
        };
    }

    if (hasRetreat) {
        jsonStruct[RETREAT] = {
            {DESIRED_DIST, grasp.post_grasp_retreat.desired_distance},
            {MIN_DIST, grasp.post_grasp_retreat.min_distance},
            {DIR_X, grasp.post_grasp_retreat.direction.vector.x},
            {DIR_Y, grasp.post_grasp_retreat.direction.vector.y},
            {DIR_Z, grasp.post_grasp_retreat.direction.vector.z},
        };
    }
    return jsonStruct;
}

bool Site::isOccupied() const {
    return isSiteOccupied;
}

void Site::setOccupied(bool isInUse) {
    isInUse = isInUse;
}
