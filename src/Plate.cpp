#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Plate.h"
#include "ServiceDefs.h"

Plate::Plate(double lengthInM, double widthInM, double heightInM) {
    plateObject.header.frame_id = PANDA_LINK_BASE;
    plateObject.id = "plate";

    plateObject.primitives.resize(1);
    plateObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    plateObject.primitives[0].dimensions.resize(3);
    plateObject.primitives[0].dimensions[0] = lengthInM;
    plateObject.primitives[0].dimensions[1] = widthInM;
    plateObject.primitives[0].dimensions[2] = heightInM;
}

Plate::Plate(const Plate& orig) {
}

Plate::~Plate() {
}

void Plate::putLocationToSite(const geometry_msgs::Pose& sitePose) {
    plateObject.primitive_poses.resize(1);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, 0.0);
    plateObject.primitive_poses[0].orientation = tf2::toMsg(orientation);
    plateObject.primitive_poses[0].position = sitePose.position;
    plateObject.operation = moveit_msgs::CollisionObject::ADD;
}

const std::string& Plate::getObjectId() const {
    return plateObject.id;
}

moveit_msgs::CollisionObject& Plate::getCollisonObject() {
    return plateObject;
}

double Plate::getLength() const {
    return plateObject.primitives[0].dimensions[0];
}

double Plate::getWidth() const {
    return plateObject.primitives[0].dimensions[1];
}

double Plate::getHeight() const {
    return plateObject.primitives[0].dimensions[2];
}
