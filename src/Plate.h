#ifndef PLATE_H
#define PLATE_H

#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nlohmann/json.hpp>
#include "Site.h"

class Plate {
public:
    // Field identifiers for JSON de-/serialization.
    static constexpr char PLATE_TYPE[] = "type";
    static constexpr char DIM_X[] = "dimX";
    static constexpr char DIM_Y[] = "dimY";
    static constexpr char DIM_Z[] = "dimZ";
    static constexpr char GRIPPER_OFFSET_X[] = "gripperOffsetX";
    static constexpr char GRIPPER_OFFSET_Y[] = "gripperOffsetY";
    static constexpr char GRIPPER_OFFSET_Z[] = "gripperOffsetZ";

    Plate(const std::string& type, double lengthInM, double widthInM, double heightInM);
    Plate(const nlohmann::json& jsonStruct);
    Plate(const Plate& orig);
    virtual ~Plate();

    void putAtSite(Site& site);
    const std::string& getId() const;
    moveit_msgs::CollisionObject& getCollisonObject();
    const std::string& getType() const;
    double getDimX() const;
    double getDimY() const;
    double getDimZ() const;
    nlohmann::json toJson() const;

private:
    moveit_msgs::CollisionObject plateObject;
    std::string type;
};

#endif /* PLATE_H */
