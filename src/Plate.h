#ifndef PLATE_H
#define PLATE_H

#include <string>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>

class Plate {
public:
    Plate(double lengthInM, double widthInM, double heightInM);
    Plate(const Plate& orig);
    virtual ~Plate();

    void putLocationToSite(const geometry_msgs::Pose& sitePose);
    const std::string& getObjectId() const;
    moveit_msgs::CollisionObject& getCollisonObject();
    double getLength() const;
    double getWidth() const;
    double getHeight() const;
private:
    moveit_msgs::CollisionObject plateObject;
};

#endif /* PLATE_H */
