#include "Plate.h"
#include "ServiceDefs.h"

Plate::Plate(const geometry_msgs::Pose& platePose) {
    plateObject.header.frame_id = PANDA_LINK_BASE;
    plateObject.id = "plate";

    plateObject.primitives.resize(1);
    plateObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    plateObject.primitives[0].dimensions.resize(3);
    plateObject.primitives[0].dimensions[0] = lengthInM;
    plateObject.primitives[0].dimensions[1] = widthInM;
    plateObject.primitives[0].dimensions[2] = heightInM;

    plateObject.primitive_poses.resize(1);
    plateObject.primitive_poses[0] = platePose;

    plateObject.operation = moveit_msgs::CollisionObject::ADD;
}

Plate::Plate(const Plate& orig) {
}

Plate::~Plate() {
}

void Plate::setPlateDimensions(double lengthInM, double widthInM, double heightInM) {
    this->lengthInM = lengthInM;
    this->widthInM = widthInM;
    this->heightInM = heightInM;
}

const std::string& Plate::getObjectId() {
    return plateObject.id;
}

moveit_msgs::CollisionObject& Plate::getCollisonObject() {
    return plateObject;
}
