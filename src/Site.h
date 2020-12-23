#ifndef SITE_H
#define SITE_H

#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Site {
public:
    Site(const geometry_msgs::Pose& pose);
    Site(const Site& orig);
    virtual ~Site();
    void setPose(const geometry_msgs::Pose& pose);
    void setApproach(const moveit_msgs::GripperTranslation& approach);
    void setRetreat(const moveit_msgs::GripperTranslation& retreat);
    moveit_msgs::Grasp& getGrasp();
private:
    std::string siteId;
    std::string description;
    moveit_msgs::Grasp grasp;
    bool hasRetreat = false;
};

#endif /* SITE_H */

