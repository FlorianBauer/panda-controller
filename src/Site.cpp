#include "Site.h"
#include "ServiceDefs.h"

Site::Site(const geometry_msgs::Pose& pose) {
    grasp.grasp_pose.pose = pose;
    grasp.grasp_pose.header.frame_id = PANDA_LINK_BASE;

    grasp.pre_grasp_approach.direction.header.frame_id = PANDA_LINK_BASE;
    grasp.pre_grasp_approach.direction.vector.x = 1.0;
    grasp.pre_grasp_approach.min_distance = 0.001;
    grasp.pre_grasp_approach.desired_distance = 0.01;

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

void Site::setPose(const geometry_msgs::Pose& pose) {
    grasp.grasp_pose.pose = pose;
}

void Site::setApproach(const moveit_msgs::GripperTranslation& approach) {
    grasp.pre_grasp_approach = approach;
    grasp.pre_grasp_approach.direction.header.frame_id = PANDA_LINK_BASE;
}

void Site::setRetreat(const moveit_msgs::GripperTranslation& retreat) {
    grasp.post_grasp_retreat = retreat;
    grasp.post_grasp_retreat.direction.header.frame_id = PANDA_LINK_BASE;
    hasRetreat = true;
}

moveit_msgs::Grasp& Site::getGrasp() {
    return grasp;
}
