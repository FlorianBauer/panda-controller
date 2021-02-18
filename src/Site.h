#ifndef SITE_H
#define SITE_H

#include <memory>
#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nlohmann/json.hpp>
#include "Plate.h"

/// Distance between EEF and gripper palm is 0.058 m + 0.002 m extra padding.
static constexpr double DIST_EEF_TO_PALM_IN_M = 0.06;
/// Default finger length in m.
static constexpr double DEFAULT_FINGER_LENGHT_IN_M = 0.11;

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

    Site(const std::string& identifier);
    Site(const nlohmann::json& jsonStruct);
    Site(const Site& orig) = default;
    Site(Site&& orig) = default;
    virtual ~Site() = default;

    static void setFingerLength(double lengthInM);
    const std::string& getId() const;
    void setPose(const geometry_msgs::Pose& pose);
    geometry_msgs::PoseStamped getPose() const;
    void setApproach(const moveit_msgs::GripperTranslation& approach);
    moveit_msgs::GripperTranslation getApproach() const;
    void setRetreat(const moveit_msgs::GripperTranslation& retreat);
    moveit_msgs::GripperTranslation getRetreat() const;
    moveit_msgs::Grasp getGrasp() const;
    moveit_msgs::PlaceLocation getPlaceLocation() const;
    nlohmann::json toJson() const;

    bool isOccupied() const;
    void setOccupied(bool isInUse);
    void putPlate(std::shared_ptr<Plate> platePtr);
    void removePlate();
    std::shared_ptr<Plate> getPlate() const;

private:
    std::string id;
    std::string description;
    moveit_msgs::Grasp grasp;
    geometry_msgs::Pose locationPose;
    bool hasApproach = false;
    bool hasRetreat = false;
    bool isSiteOccupied = false;
    std::shared_ptr<Plate> platePtr;
};

#endif /* SITE_H */
