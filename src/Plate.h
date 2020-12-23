#ifndef PLATE_H
#define PLATE_H

#include <string>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

class Plate {
public:
    Plate(const geometry_msgs::Pose& platePose);
    Plate(const Plate& orig);
    virtual ~Plate();
    void setPlateDimensions(double lengthInM, double widthInM, double heightInM);
    const std::string& getObjectId();
    moveit_msgs::CollisionObject& getCollisonObject();
private:
    double lengthInM = 0.0;
    double widthInM = 0.0;
    double heightInM = 0.0;
    moveit_msgs::CollisionObject plateObject;
};

#endif /* PLATE_H */
