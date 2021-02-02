#ifndef PLATE_H
#define PLATE_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nlohmann/json.hpp>
#include "Site.h"

class Plate {
public:
    // Field identifiers for JSON de-/serialization.
    static constexpr char PLATE_TYPE_ID[] = "typeId";
    static constexpr char DIM_X[] = "dimX";
    static constexpr char DIM_Y[] = "dimY";
    static constexpr char DIM_Z[] = "dimZ";
    static constexpr char GRIPPER_OFFSET_X[] = "gripperOffsetX";
    static constexpr char GRIPPER_OFFSET_Y[] = "gripperOffsetY";
    static constexpr char GRIPPER_OFFSET_Z[] = "gripperOffsetZ";

    Plate(const std::string& typeId, double lengthInM, double widthInM, double heightInM);
    Plate(const nlohmann::json& jsonStruct);
    Plate(const Plate& orig) = default;
    virtual ~Plate() = default;

    void putAtSite(Site& site);
    const std::string& getObjectId() const;
    const std::string& getTypeId() const;
    moveit_msgs::CollisionObject& getCollisonObject();
    void setPosition(const geometry_msgs::Pose::_position_type& position);
    void setOrientation(const geometry_msgs::Pose::_orientation_type& orientation);
    double getDimX() const;
    double getDimY() const;
    double getDimZ() const;
    nlohmann::json toJson() const;

private:
    moveit_msgs::CollisionObject plateObject;
    std::string typeId;
    double offX = 0.0;
    double offY = 0.0;
    double offZ = 0.0;
};

#endif /* PLATE_H */
