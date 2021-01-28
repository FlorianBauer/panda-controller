#ifndef SITE_H
#define SITE_H

#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nlohmann/json.hpp>

class Site {
public:
    // Field identifiers for JSON de-/serialization.
    static constexpr char SITE_ID[] = "id";
    static constexpr char POSE[] = "pose";
    static constexpr char POS_X[] = "posX";
    static constexpr char POS_Y[] = "posY";
    static constexpr char POS_Z[] = "posZ";
    static constexpr char ORI_X[] = "oriX";
    static constexpr char ORI_Y[] = "oriY";
    static constexpr char ORI_Z[] = "oriZ";
    static constexpr char ORI_W[] = "oriW";
    static constexpr char APPROACH[] = "approach";
    static constexpr char RETREAT[] = "retreat";
    static constexpr char DESIRED_DIST[] = "desiredDist";
    static constexpr char MIN_DIST[] = "minDist";
    static constexpr char DIR_X[] = "dirX";
    static constexpr char DIR_Y[] = "dirY";
    static constexpr char DIR_Z[] = "dirZ";

    Site(const std::string& identifier, const geometry_msgs::Pose& pose);
    Site(const nlohmann::json& jsonStruct);
    Site(const Site& orig);
    virtual ~Site();

    const std::string& getId() const;
    geometry_msgs::PoseStamped getPose() const;
    moveit_msgs::GripperTranslation getApproach() const;
    moveit_msgs::GripperTranslation getRetreat() const;
    moveit_msgs::Grasp getGrasp() const;
    moveit_msgs::PlaceLocation getPlaceLocation() const;
    nlohmann::json toJson() const;
    void setApproach(const moveit_msgs::GripperTranslation& approach);
    void setRetreat(const moveit_msgs::GripperTranslation& retreat);
    bool isOccupied() const;
    void setOccupied(bool isInUse);

private:
    std::string id;
    std::string description;
    moveit_msgs::Grasp grasp;
    geometry_msgs::Pose locationPose;
    bool hasApproach = false;
    bool hasRetreat = false;
    bool isSiteOccupied = false;
};

#endif /* SITE_H */
