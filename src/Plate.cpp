/**
 * Class for Plates describing the transportable object and its dimensions for grabbing. The JSON 
 * constructor is capable of using parsed Opentrons labware files as input. Definitions of common 
 * used labware can be found here:
 * https://github.com/Opentrons/opentrons/tree/master/shared-data/labware/definitions/2
 */
#include "Plate.h"
#include "ServiceDefs.h"

using json = nlohmann::json;

// Field identifiers for de-/serialization.
static constexpr char PLATE_METADATA[] = "metadata";
static constexpr char PLATE_DSIP_NAME[] = "displayName";
static constexpr char PLATE_PARAMETERS[] = "parameters";
static constexpr char PLATE_LOAD_NAME[] = "loadName";
static constexpr char PLATE_DIM[] = "dimensions";
static constexpr char DIM_X[] = "xDimension";
static constexpr char DIM_Y[] = "yDimension";
static constexpr char DIM_Z[] = "zDimension";

/// ID counter
static unsigned idCounter = 0;

/**
 * Constructor.
 * 
 * @param loadName A string describing the type of the object (e.g. "corning_96_wellplate_360ul_flat").
 * @param dimX The length of the object in meter.
 * @param dimY The width of the object in meter.
 * @param dimZ The height of the object in meter.
 */
Plate::Plate(const std::string& typeName, double dimX, double dimY, double dimZ) {
    loadName = typeName;
    plateObject.id = typeName + std::to_string(++idCounter);
    plateObject.header.frame_id = PANDA_LINK_BASE;

    plateObject.primitives.resize(1);
    plateObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    plateObject.primitives[0].dimensions.resize(3);
    plateObject.primitives[0].dimensions[0] = dimX;
    plateObject.primitives[0].dimensions[1] = dimY;
    plateObject.primitives[0].dimensions[2] = dimZ;
}

/**
 * Constructor for creation from a JSON struct. Parsed Opentrons labware files can be used as input.
 * The definitions therefore can be found here: 
 * https://github.com/Opentrons/opentrons/tree/master/shared-data/labware/definitions/2
 * 
 * @param jsonStruct The JSON struct to read the data from.
 */
Plate::Plate(const json& jsonStruct) {
    loadName = jsonStruct[PLATE_PARAMETERS][PLATE_LOAD_NAME].get<std::string>();
    plateObject.id = loadName + std::to_string(++idCounter);
    plateObject.header.frame_id = PANDA_LINK_BASE;

    plateObject.primitives.resize(1);
    plateObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    plateObject.primitives[0].dimensions.resize(3);
    const json& jsonDims = jsonStruct[PLATE_DIM];
    plateObject.primitives[0].dimensions[0] = jsonDims[DIM_X].get<double>() * 100.0;
    plateObject.primitives[0].dimensions[1] = jsonDims[DIM_Y].get<double>() * 100.0;
    plateObject.primitives[0].dimensions[2] = jsonDims[DIM_Z].get<double>() * 100.0;
}

/**
 * Copy-Constructor.
 */
Plate::Plate(const Plate& orig) {
}

/**
 * Destructor.
 */
Plate::~Plate() {
}

/**
 * Sets the position of this plate to the corresponding `Site`. The `Site` is then marked as 
 * occupied.
 * 
 * @param site The `Site` to place this plate at.
 */
void Plate::putAtSite(Site& site) {
    site.setOccupied(true);
    plateObject.primitive_poses.resize(1);
    const geometry_msgs::PoseStamped& pose = site.getSitePose();
    // The orientation is just meant for the gripper, so we ignore it and use just the position.
    plateObject.primitive_poses[0].position = pose.pose.position;
    plateObject.operation = moveit_msgs::CollisionObject::ADD;
}

/**
 * Get the unique identifier of this object.
 * 
 * @return The identifier as string.
 */
const std::string& Plate::getPlateId() const {
    return plateObject.id;
}

/**
 * Get the name of the plate type.
 * 
 * @return The type as string.
 */
const std::string& Plate::getTypeName() const {
    return loadName;
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
 * Get the Plate data as JSON struct. The dimensions of the object are therefore converted from
 * meter into millimeter.
 * 
 * @return The JSON struct.
 */
json Plate::getPlateAsJson() const {
    json jsonStruct;
    jsonStruct[PLATE_PARAMETERS] = {PLATE_LOAD_NAME, loadName};
    jsonStruct[PLATE_DIM] = {
        {DIM_X, plateObject.primitives[0].dimensions[0] / 100.0},
        {DIM_Y, plateObject.primitives[0].dimensions[1] / 100.0},
        {DIM_Z, plateObject.primitives[0].dimensions[2] / 100.0},
    };
    return jsonStruct;
}