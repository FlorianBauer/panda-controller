//============================================================================
/// \file    RobotControllerImpl.h
/// \authors Florian Bauer <florian.bauer.dev@gmail.com>
/// \date    2021-01-26
/// \brief   Declaration of the CRobotControllerImpl class
/// \note    Code generated by sila2codegenerator 0.3.3-dev
//============================================================================
#ifndef ROBOTCONTROLLERIMPL_H
#define ROBOTCONTROLLERIMPL_H

#include <memory>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sila_cpp/server/SiLAFeature.h>
#include <sila_cpp/data_types.h>
#include <sila_cpp/server/command/UnobservableCommand.h>
#include <sila_cpp/server/command/ObservableCommand.h>
#include <sila_cpp/server/property/UnobservableProperty.h>
#include "Pose.h"
#include "RelativeMove.h"
#include "RobotController.grpc.pb.h"
#include "ServiceDefs.h"
#include "Site.h"
#include "SiteManager/SiteManagerImpl.h"

/**
 * @brief The CRobotControllerImpl class implements the RobotController feature
 *
 * @details Controller for a Panda robot arm.
 */
class CRobotControllerImpl final : public SiLA2::CSiLAFeature<sila2::de::fau::robot::robotcontroller::v1::RobotController> {
    // Using declarations for the Feature's Commands and Properties
    using GetCurrentFrameCommand =
            SiLA2::CUnobservableCommandManager<&CRobotControllerImpl::RequestGetCurrentFrame>;
    using GetCurrentFrameWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::GetCurrentFrame_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::GetCurrentFrame_Responses>;
    using GetCurrentPoseCommand =
            SiLA2::CUnobservableCommandManager<&CRobotControllerImpl::RequestGetCurrentPose>;
    using GetCurrentPoseWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::GetCurrentPose_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::GetCurrentPose_Responses>;
    using MoveToPoseCommand =
            SiLA2::CUnobservableCommandManager<&CRobotControllerImpl::RequestMoveToPose>;
    using MoveToPoseWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::MoveToPose_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::MoveToPose_Responses>;
    using MoveToSiteCommand =
            SiLA2::CUnobservableCommandManager<&CRobotControllerImpl::RequestMoveToSite>;
    using MoveToSiteWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::MoveToSite_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::MoveToSite_Responses>;
    using MoveRelativeCommand =
            SiLA2::CUnobservableCommandManager<&CRobotControllerImpl::RequestMoveRelative>;
    using MoveRelativeWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::MoveRelative_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::MoveRelative_Responses>;
    using TransportPlateCommand = SiLA2::CObservableCommandManager<
            &CRobotControllerImpl::RequestTransportPlate,
            &CRobotControllerImpl::RequestTransportPlate_Info,
            &CRobotControllerImpl::RequestTransportPlate_Result>;
    using TransportPlateWrapper = SiLA2::CObservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::TransportPlate_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::TransportPlate_Responses>;
    using PickPlateCommand = SiLA2::CObservableCommandManager<
            &CRobotControllerImpl::RequestPickPlate,
            &CRobotControllerImpl::RequestPickPlate_Info,
            &CRobotControllerImpl::RequestPickPlate_Result>;
    using PickPlateWrapper = SiLA2::CObservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::PickPlate_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::PickPlate_Responses>;
    using PlacePlateCommand = SiLA2::CObservableCommandManager<
            &CRobotControllerImpl::RequestPlacePlate,
            &CRobotControllerImpl::RequestPlacePlate_Info,
            &CRobotControllerImpl::RequestPlacePlate_Result>;
    using PlacePlateWrapper = SiLA2::CObservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::PlacePlate_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::PlacePlate_Responses>;
    using IsSiteOccupiedCommand =
            SiLA2::CUnobservableCommandManager<&CRobotControllerImpl::RequestIsSiteOccupied>;
    using IsSiteOccupiedWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::IsSiteOccupied_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::IsSiteOccupied_Responses>;
    using FollowPathCommand = SiLA2::CObservableCommandManager<
            &CRobotControllerImpl::RequestFollowPath,
            &CRobotControllerImpl::RequestFollowPath_Info,
            &CRobotControllerImpl::RequestFollowPath_Result>;
    using FollowPathWrapper = SiLA2::CObservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::FollowPath_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::FollowPath_Responses>;
    using SetToFrameCommand =
            SiLA2::CUnobservableCommandManager<&CRobotControllerImpl::RequestSetToFrame>;
    using SetToFrameWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::SetToFrame_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::SetToFrame_Responses>;
    using FollowFramesCommand = SiLA2::CObservableCommandManager<
            &CRobotControllerImpl::RequestFollowFrames,
            &CRobotControllerImpl::RequestFollowFrames_Info,
            &CRobotControllerImpl::RequestFollowFrames_Result>;
    using FollowFramesWrapper = SiLA2::CObservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::FollowFrames_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::FollowFrames_Responses>;
    using SetGripperCommand =
            SiLA2::CUnobservableCommandManager<&CRobotControllerImpl::RequestSetGripper>;
    using SetGripperWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::SetGripper_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::SetGripper_Responses>;
    using CloseGripperCommand =
            SiLA2::CUnobservableCommandManager<&CRobotControllerImpl::RequestCloseGripper>;
    using CloseGripperWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::robotcontroller::v1::CloseGripper_Parameters,
            sila2::de::fau::robot::robotcontroller::v1::CloseGripper_Responses>;

public:
    /**
     * @brief C'tor
     *
     * @param parent The SiLA server instance that contains this Feature
     */
    explicit CRobotControllerImpl(SiLA2::CSiLAServer* parent, const std::shared_ptr<CSiteManagerImpl> siteManagerPtr);

    /**
     * @brief GetCurrentFrame Command
     *
     * @details Get the current joint values.
     *
     * @param Command The current GetCurrentFrame Command Execution Wrapper
     * It contains the following Parameters:
     * None
     *
     * @return GetCurrentFrame_Responses The Command Response
     * It contains the following fields:
     * @li Frame The absolute joint values.
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::GetCurrentFrame_Responses GetCurrentFrame(GetCurrentFrameWrapper* command);

    /**
     * @brief GetCurrentPose Command
     *
     * @details Get the current position and orientation of the robot hand.
     *
     * @param Command The current GetCurrentPose Command Execution Wrapper
     * It contains the following Parameters:
     * None
     *
     * @return GetCurrentPose_Responses The Command Response
     * It contains the following fields:
     * @li Pose The pose to query.
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::GetCurrentPose_Responses GetCurrentPose(GetCurrentPoseWrapper* command);

    /**
     * @brief MoveToPose Command
     *
     * @details Move to a given position and with a given orientation.
     *
     * @param Command The current MoveToPose Command Execution Wrapper
     * It contains the following Parameters:
     * @li Pose The pose to move to.
     *
     * @return MoveToPose_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::MoveToPose_Responses MoveToPose(MoveToPoseWrapper* command);

    /**
     * @brief MoveToSite Command
     *
     * @details Move to a given site without approaching.
     *
     * @param Command The current MoveToSite Command Execution Wrapper
     * It contains the following Parameters:
     * @li SiteId Site to move to.
     *
     * @return MoveToSite_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::MoveToSite_Responses MoveToSite(MoveToSiteWrapper* command);

    /**
     * @brief MoveRelative Command
     *
     * @details Move in the given direction without changing the current orientation of the
     * robot hand.
     *
     * @param Command The current MoveRelative Command Execution Wrapper
     * It contains the following Parameters:
     * @li RelativeMove The position changing move operation.
     *
     * @return MoveRelative_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::MoveRelative_Responses MoveRelative(MoveRelativeWrapper* command);

    /**
     * @brief TransportPlate Command
     *
     * @details Transports a plate between the two given sites.
     *
     * @param Command The current TransportPlate Command Execution Wrapper
     * It contains the following Parameters:
     * @li OriginSiteId Site to move the plate from.
     * @li DestinationSiteId Site to move the plate to.
     * @li PlateType The plate type to grab.
     *
     * @return TransportPlate_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::TransportPlate_Responses TransportPlate(TransportPlateWrapper* command);

    /**
     * @brief PickPlate Command
     *
     * @details Pick up a plate from a given site.
     *
     * @param Command The current PickPlate Command Execution Wrapper
     * It contains the following Parameters:
     * @li SiteId Site to pick plate from.
     * @li PlateType The plate type to grab.
     *
     * @return PickPlate_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::PickPlate_Responses PickPlate(PickPlateWrapper* command);

    /**
     * @brief PlacePlate Command
     *
     * @details Place a plate on a given site.
     *
     * @param Command The current PlacePlate Command Execution Wrapper
     * It contains the following Parameters:
     * @li SiteId Site to place the plate on.
     * @li PlateType The plate type to grab.
     *
     * @return PlacePlate_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::PlacePlate_Responses PlacePlate(PlacePlateWrapper* command);

    /**
     * @brief IsSiteOccupied Command
     *
     * @details Check if the given site is currently occupied with a sample.
     *
     * @param Command The current IsSiteOccupied Command Execution Wrapper
     * It contains the following Parameters:
     * @li SiteId The Site to check.
     *
     * @return IsSiteOccupied_Responses The Command Response
     * It contains the following fields:
     * @li IsOccupied Boolean describing if site is occupied or not.
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::IsSiteOccupied_Responses IsSiteOccupied(IsSiteOccupiedWrapper* command);

    /**
     * @brief FollowPath Command
     *
     * @details Follows a path of poses.
     *
     * @param Command The current FollowPath Command Execution Wrapper
     * It contains the following Parameters:
     * @li PoseList A list of poses to follow along.
     *
     * @return FollowPath_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::FollowPath_Responses FollowPath(FollowPathWrapper* command);

    /**
     * @brief SetToFrame Command
     *
     * @details Sets the absolute joint values.
     *
     * @param Command The current SetToFrame Command Execution Wrapper
     * It contains the following Parameters:
     * @li Frame The frame to set the joints to.
     *
     * @return SetToFrame_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::SetToFrame_Responses SetToFrame(SetToFrameWrapper* command);

    /**
     * @brief FollowFrames Command
     *
     * @details Successively set the absolute joint values.
     *
     * @param Command The current FollowFrames Command Execution Wrapper
     * It contains the following Parameters:
     * @li FrameList The list of frames to sequence.
     *
     * @return FollowFrames_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::FollowFrames_Responses FollowFrames(FollowFramesWrapper* command);

    /**
     * @brief SetGripper Command
     *
     * @details Sets the distance between the two finger to the given width. This operation can
     * be used to open or close the gripper.
     *
     * @param Command The current SetGripper Command Execution Wrapper
     * It contains the following Parameters:
     * @li Width The width between the two finger.
     *
     * @return SetGripper_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::SetGripper_Responses SetGripper(SetGripperWrapper* command);

    /**
     * @brief CloseGripper Command
     *
     * @details Closes the gripper.
     *
     * @param Command The current CloseGripper Command Execution Wrapper
     * It contains the following Parameters:
     * None
     *
     * @return CloseGripper_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::robotcontroller::v1::CloseGripper_Responses CloseGripper(CloseGripperWrapper* command);

private:
    const std::shared_ptr<CSiteManagerImpl> m_SiteManagerPtr;
    GetCurrentFrameCommand m_GetCurrentFrameCommand;
    GetCurrentPoseCommand m_GetCurrentPoseCommand;
    MoveToPoseCommand m_MoveToPoseCommand;
    MoveToSiteCommand m_MoveToSiteCommand;
    MoveRelativeCommand m_MoveRelativeCommand;
    TransportPlateCommand m_TransportPlateCommand;
    PickPlateCommand m_PickPlateCommand;
    PlacePlateCommand m_PlacePlateCommand;
    IsSiteOccupiedCommand m_IsSiteOccupiedCommand;
    FollowPathCommand m_FollowPathCommand;
    SetToFrameCommand m_SetToFrameCommand;
    FollowFramesCommand m_FollowFramesCommand;
    SetGripperCommand m_SetGripperCommand;
    CloseGripperCommand m_CloseGripperCommand;
    ros::NodeHandle m_RosNode;
    moveit::planning_interface::MoveGroupInterface m_Arm{PANDA_ARM};
    moveit::planning_interface::MoveGroupInterface m_Gripper{PANDA_HAND};
    moveit::planning_interface::PlanningSceneInterface m_PlanningScene;

    void openGripper(trajectory_msgs::JointTrajectory& posture, double width);
    void closeGripper(trajectory_msgs::JointTrajectory& posture);
};

#endif  // ROBOTCONTROLLERIMPL_H
