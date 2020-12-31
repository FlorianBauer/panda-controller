#ifndef SITE_H
#define SITE_H

#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nlohmann/json.hpp>

class Site {
public:
    Site(const std::string& identifier, const geometry_msgs::Pose& pose);
    Site(const nlohmann::json& jsonStruct);
    Site(const Site& orig);
    virtual ~Site();

    const std::string& getSiteId() const;
    geometry_msgs::PoseStamped getSitePose() const;
    void setApproach(const moveit_msgs::GripperTranslation& approach);
    moveit_msgs::GripperTranslation getApproach() const;
    void setRetreat(const moveit_msgs::GripperTranslation& retreat);
    moveit_msgs::GripperTranslation getRetreat() const;
    moveit_msgs::Grasp& getGrasp();
    moveit_msgs::PlaceLocation getPlaceLocation() const;
    nlohmann::json getSiteAsJson() const;
    bool isOccupied() const;
    void setOccupied(bool isInUse);

private:
    std::string siteId;
    std::string description;
    moveit_msgs::Grasp grasp;
    geometry_msgs::Pose locationPose;
    bool hasApproach = false;
    bool hasRetreat = false;
    bool isSiteOccupied = false;
};

#endif /* SITE_H */
