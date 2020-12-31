#include "Plate.h"
#include "ServiceDefs.h"

using json = nlohmann::json;

// Field identifiers for de-/serialization.
static constexpr char PLATE_TYPE[] = "type";
static constexpr char SIZE_X[] = "sizeX";
static constexpr char SIZE_Y[] = "sizeY";
static constexpr char SIZE_Z[] = "sizeZ";

/// ID counter
static unsigned id = 0;

/**
 * Constructor.
 * 
 * @param type A string describing the type of the object (e.g. "wells_96").
 * @param lengthInM The length of the object in meter.
 * @param widthInM The width of the object in meter.
 * @param heightInM Te height of the object in meter.
 */
Plate::Plate(const std::string& type, double lengthInM, double widthInM, double heightInM) {
    this->type = type;
    plateObject.id = type + std::to_string(++id);
    plateObject.header.frame_id = PANDA_LINK_BASE;

    plateObject.primitives.resize(1);
    plateObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    plateObject.primitives[0].dimensions.resize(3);
    plateObject.primitives[0].dimensions[0] = lengthInM;
    plateObject.primitives[0].dimensions[1] = widthInM;
    plateObject.primitives[0].dimensions[2] = heightInM;
}

/**
 * Constructor for creation from a JSON struct.
 * 
 * @param jsonStruct The JSON struct to read the data from.
 */
Plate::Plate(const json& jsonStruct) {
    type = jsonStruct[PLATE_TYPE].get<std::string>();
    plateObject.id = type + std::to_string(++id);
    plateObject.header.frame_id = PANDA_LINK_BASE;

    plateObject.primitives.resize(1);
    plateObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    plateObject.primitives[0].dimensions.resize(3);
    plateObject.primitives[0].dimensions[0] = jsonStruct[SIZE_X].get<double>();
    plateObject.primitives[0].dimensions[1] = jsonStruct[SIZE_Y].get<double>();
    plateObject.primitives[0].dimensions[2] = jsonStruct[SIZE_Z].get<double>();
}

Plate::Plate(const Plate& orig) {
}

Plate::~Plate() {
}

/**
 * Sets the orientation and position of this plate to the corresponding Site. The Site is then 
 * marked as occupied.
 * 
 * @param site The Site to place this plate at.
 */
void Plate::putAtSite(Site& site) {
    site.setOccupied(true);
    plateObject.primitive_poses.resize(1);
    const geometry_msgs::PoseStamped& pose = site.getSitePose();
    plateObject.primitive_poses[0].orientation = pose.pose.orientation;
    plateObject.primitive_poses[0].position = pose.pose.position;
    plateObject.operation = moveit_msgs::CollisionObject::ADD;
}

const std::string& Plate::getPlateId() const {
    return plateObject.id;
}

const std::string& Plate::getType() const {
    return type;
}

moveit_msgs::CollisionObject& Plate::getCollisonObject() {
    return plateObject;
}

double Plate::getSizeX() const {
    return plateObject.primitives[0].dimensions[0];
}

double Plate::getSizeY() const {
    return plateObject.primitives[0].dimensions[1];
}

double Plate::getSizeZ() const {
    return plateObject.primitives[0].dimensions[2];
}

json Plate::getPlateAsJson() const {
    json jsonStruct;
    jsonStruct[PLATE_TYPE] = type;
    jsonStruct[SIZE_X] = plateObject.primitives[0].dimensions[0];
    jsonStruct[SIZE_Y] = plateObject.primitives[0].dimensions[1];
    jsonStruct[SIZE_Z] = plateObject.primitives[0].dimensions[2];
    return jsonStruct;
}