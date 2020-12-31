#ifndef PLATE_H
#define PLATE_H

#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nlohmann/json.hpp>
#include "Site.h"

class Plate {
public:
    Plate(const std::string& type, double lengthInM, double widthInM, double heightInM);
    Plate(const nlohmann::json& jsonStruct);
    Plate(const Plate& orig);
    virtual ~Plate();

    void putAtSite(Site& site);
    const std::string& getPlateId() const;
    moveit_msgs::CollisionObject& getCollisonObject();
    const std::string& getType() const;
    double getSizeX() const;
    double getSizeY() const;
    double getSizeZ() const;
    nlohmann::json getPlateAsJson() const;

private:
    moveit_msgs::CollisionObject plateObject;
    std::string type;
};

#endif /* PLATE_H */
