#ifndef MICROPLATETRANSPORT_H
#define MICROPLATETRANSPORT_H

//#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MicroplateTransport {
public:
    static constexpr double MICROPLATE_LENGTH_IN_M = 0.12776;
    static constexpr double MICROPLATE_WIDTH_IN_M = 0.08548;
    static constexpr double MICROPLATE_HEIGHT_IN_M = 0.01435;

    MicroplateTransport();
    MicroplateTransport(const MicroplateTransport& orig);
    virtual ~MicroplateTransport();

    void run();
    void loadFrankaEmikaTaskFile();
    
private:
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closeGripper(trajectory_msgs::JointTrajectory& posture);
};

#endif /* MICROPLATETRANSPORT_H */
