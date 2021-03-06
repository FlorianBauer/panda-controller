/**
 * Class for Plates describing the transportable object and its dimensions and offsets for gripping.
 */
#include "Plate.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ServiceDefs.h"

using json = nlohmann::json;

/// ID counter
static unsigned instanceIdCounter = 0;

/**
 * Constructor.
 * 
 * @param loadName A string describing the type of the object (e.g. "corning_96_wellplate_360ul_flat").
 * @param dimX The length of the object in meter.
 * @param dimY The width of the object in meter.
 * @param dimZ The height of the object in meter.
 */
Plate::Plate(const std::string& typeName, double dimX, double dimY, double dimZ) {
    typeId = typeName;
    plateObject.id = typeName + "_" + std::to_string(++instanceIdCounter);
    plateObject.header.frame_id = PANDA_LINK_BASE;
    plateObject.primitive_poses.resize(1);
    tf2::Quaternion orientation;
    plateObject.primitive_poses[0].orientation = tf2::toMsg(orientation);

    plateObject.primitives.resize(1);
    plateObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    plateObject.primitives[0].dimensions.resize(3);
    plateObject.primitives[0].dimensions[0] = dimX;
    plateObject.primitives[0].dimensions[1] = dimY;
    plateObject.primitives[0].dimensions[2] = dimZ;
}

/**
 * Constructor for creation from a JSON structure.
 * 
 * @param jsonStruct The JSON struct to read the data from.
 */
Plate::Plate(const json& jsonStruct) {
    typeId = jsonStruct[PLATE_TYPE_ID].get<std::string>();
    plateObject.id = typeId + "_" + std::to_string(++instanceIdCounter);
    plateObject.header.frame_id = PANDA_LINK_BASE;
    plateObject.primitive_poses.resize(1);
    tf2::Quaternion orientation;
    plateObject.primitive_poses[0].orientation = tf2::toMsg(orientation);

    plateObject.primitives.resize(1);
    plateObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    plateObject.primitives[0].dimensions.resize(3);
    plateObject.primitives[0].dimensions[0] = jsonStruct[DIM_X].get<double>();
    plateObject.primitives[0].dimensions[1] = jsonStruct[DIM_Y].get<double>();
    plateObject.primitives[0].dimensions[2] = jsonStruct[DIM_Z].get<double>();
    offX = jsonStruct[GRIPPER_OFFSET_X].get<double>();
    offY = jsonStruct[GRIPPER_OFFSET_Y].get<double>();
    offZ = jsonStruct[GRIPPER_OFFSET_Z].get<double>();
}

/**
 * Get the unique identifier of this object.
 * 
 * @return The identifier as string.
 */
const std::string& Plate::getObjectId() const {
    return plateObject.id;
}

/**
 * Get the name of the plate type.
 * 
 * @return The type as string.
 */
const std::string& Plate::getTypeId() const {
    return typeId;
}

/**
 * Get the `CollisionObject` for the gripper.
 * 
 * @return The `CollisionObject`.
 */
moveit_msgs::CollisionObject& Plate::getCollisonObject() {
    return plateObject;
}

/**
 * Set the position of the `CollisionObject`.
 * 
 * @param position The new position of the `CollisionObject`.
 */
void Plate::setPosition(const geometry_msgs::Pose::_position_type& position) {
    plateObject.primitive_poses[0].position = position;
}

/**
 * Set the orientation of the `CollisionObject`.
 * 
 * @param orientation The new orientation of the `CollisionObject`.
 */
void Plate::setOrientation(const geometry_msgs::Pose::_orientation_type& orientation) {
    plateObject.primitive_poses[0].orientation = orientation;
}

/**
 * Get the length of the object.
 * 
 * @return The length in meter.
 */
double Plate::getDimX() const {
    return plateObject.primitives[0].dimensions[0];
}

/**
 * Get the width of the object.
 * 
 * @return The width in meter.
 */
double Plate::getDimY() const {
    return plateObject.primitives[0].dimensions[1];
}

/**
 * Get the height of the object.
 * 
 * @return The height in meter.
 */
double Plate::getDimZ() const {
    return plateObject.primitives[0].dimensions[2];
}

/**
 * Get the Plate data as JSON struct.
 * 
 * @return The JSON struct.
 */
json Plate::toJson() const {
    json jsonStruct;
    jsonStruct[PLATE_TYPE_ID] = typeId;
    jsonStruct[DIM_X] = plateObject.primitives[0].dimensions[0];
    jsonStruct[DIM_Y] = plateObject.primitives[0].dimensions[1];
    jsonStruct[DIM_Z] = plateObject.primitives[0].dimensions[2];
    jsonStruct[GRIPPER_OFFSET_X] = offX;
    jsonStruct[GRIPPER_OFFSET_Y] = offY;
    jsonStruct[GRIPPER_OFFSET_Z] = offZ;
    return jsonStruct;
}