//============================================================================
/// \file    RobotControllerImpl.cpp
/// \authors Florian Bauer <florian.bauer.dev@gmail.com>
/// \date    2021-01-20
/// \brief   Definition of the CRobotControllerImpl class
/// \note    Code generated by sila2codegenerator 0.3.3-dev
//============================================================================

#include "RobotControllerImpl.h"

#include <fstream>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nlohmann/json.hpp>
#include <sila_cpp/common/logging.h>
#include <sila_cpp/framework/error_handling/ExecutionError.h>
#include "FileManager.h"
#include "RobotController.pb.h"

using namespace sila2::de::fau::robot::robotcontroller::v1;
using moveit::planning_interface::MoveItErrorCode;

CRobotControllerImpl::CRobotControllerImpl(SiLA2::CSiLAServer* parent, const std::shared_ptr<CSiteManagerImpl> siteManagerPtr)
: CSiLAFeature{parent},
m_SiteManagerPtr{siteManagerPtr},
m_GetCurrentFrameCommand{this, "GetCurrentFrame"},
m_GetCurrentPoseCommand{this, "GetCurrentPose"},
m_MoveToPoseCommand{this, "MoveToPose"},
m_MoveToSiteCommand{this, "MoveToSite"},
m_RetreatCommand{this, "Retreat"},
m_ApproachToCommand{this, "ApproachTo"},
m_TransportPlateCommand{this, "TransportPlate"},
m_PickPlateCommand{this, "PickPlate"},
m_PlacePlateCommand{this, "PlacePlate"},
m_CheckOccupiedCommand{this, "CheckOccupied"},
m_FollowPathCommand{this, "FollowPath"},
m_SetToFrameCommand{this, "SetToFrame"},
m_FollowFramesCommand{this, "FollowFrames"}
{
    m_GetCurrentFrameCommand.setExecutor(this, &CRobotControllerImpl::GetCurrentFrame);
    m_GetCurrentPoseCommand.setExecutor(this, &CRobotControllerImpl::GetCurrentPose);
    m_MoveToPoseCommand.setExecutor(this, &CRobotControllerImpl::MoveToPose);
    m_MoveToSiteCommand.setExecutor(this, &CRobotControllerImpl::MoveToSite);
    m_RetreatCommand.setExecutor(this, &CRobotControllerImpl::Retreat);
    m_ApproachToCommand.setExecutor(this, &CRobotControllerImpl::ApproachTo);
    m_TransportPlateCommand.setExecutor(this, &CRobotControllerImpl::TransportPlate);
    m_PickPlateCommand.setExecutor(this, &CRobotControllerImpl::PickPlate);
    m_PlacePlateCommand.setExecutor(this, &CRobotControllerImpl::PlacePlate);
    m_CheckOccupiedCommand.setExecutor(this, &CRobotControllerImpl::CheckOccupied);
    m_FollowPathCommand.setExecutor(this, &CRobotControllerImpl::FollowPath);
    m_SetToFrameCommand.setExecutor(this, &CRobotControllerImpl::SetToFrame);
    m_FollowFramesCommand.setExecutor(this, &CRobotControllerImpl::FollowFrames);
}

/**
 * Get the current frame of the robot. A Frame consists of the absolute values of all joints.
 * 
 * @param command The SiLA command.
 * @return The values of all robot joints.
 */
GetCurrentFrame_Responses CRobotControllerImpl::GetCurrentFrame(GetCurrentFrameWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;

    std::vector<double> joints = m_MoveGroup.getCurrentJointValues();
    auto response = GetCurrentFrame_Responses{};
    const auto frame = response.mutable_frame();
    for (size_t i = 0; i < MAX_JOINTS; i++) {
        const auto joint = frame->add_frame();
        joint->set_value(joints[i]);
    }
    return response;
}

/**
 * Get the current Pose (position + orientation as quaternion) of the robot hand.
 * 
 * @param command The SiLA command.
 * @return The pose consist of position (xyz) and orientation (xyzw).
 */
GetCurrentPose_Responses CRobotControllerImpl::GetCurrentPose(GetCurrentPoseWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;

    const geometry_msgs::PoseStamped curPose = m_MoveGroup.getCurrentPose(PANDA_LINK_HAND);
    const Pose retPose = {
        .X = curPose.pose.position.x,
        .Y = curPose.pose.position.y,
        .Z = curPose.pose.position.z,
        .OriX = curPose.pose.orientation.x,
        .OriY = curPose.pose.orientation.y,
        .OriZ = curPose.pose.orientation.z,
        .OriW = curPose.pose.orientation.w
    };

    auto response = GetCurrentPose_Responses{};
    response.set_allocated_pose(retPose.toProtoMessagePtr());
    return response;
}

/**
 * Moves the robot hand to the given pose. Since path planning is active on this request, an invalid 
 * pose or an pose causing a collision will throw a validation error.
 * 
 * @param command The SiLA command.
 * @return A empty response.
 */
MoveToPose_Responses CRobotControllerImpl::MoveToPose(MoveToPoseWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;

    const auto& pose = request.pose().pose();
    geometry_msgs::Pose targetPose;
    targetPose.position.x = pose.x().value();
    targetPose.position.y = pose.y().value();
    targetPose.position.z = pose.z().value();
    targetPose.orientation.x = pose.orix().value();
    targetPose.orientation.y = pose.oriy().value();
    targetPose.orientation.z = pose.oriz().value();
    targetPose.orientation.w = pose.oriw().value();
    m_MoveGroup.setPoseTarget(targetPose);
    const MoveItErrorCode err = m_MoveGroup.move();

    if (err != MoveItErrorCode::SUCCESS) {
        throw SiLA2::CDefinedExecutionError{
            "InvalidPose",
            "The given pose is invalid, not within reach or would cause a collision."};
    }

    return MoveToPose_Responses{};
}

MoveToSite_Responses CRobotControllerImpl::MoveToSite(MoveToSiteWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;

    if (!m_SiteManagerPtr->hasSiteId(request.siteid().siteid().value())) {
        // Throw an validation exception if site is not available.
        throw SiLA2::CDefinedExecutionError{
            "SiteIdNotFound",
            "The given Site ID does not exist or could not be found."};
    }

    // TODO: Write actual Command implementation logic...
    const double NUM_STEPS = 10.0;
    for (int i = 0; i <= NUM_STEPS; ++i) {
        // do stuff...
        command->setExecutionInfo(SiLA2::CReal{i / NUM_STEPS});
    }

    auto Response = MoveToSite_Responses{};
    // TODO: Fill the response fields
    return Response;
}

Retreat_Responses CRobotControllerImpl::Retreat(RetreatWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;
    // TODO: Validate request parameters...

    // TODO: Write actual Command implementation logic...
    const double NUM_STEPS = 10.0;
    for (int i = 0; i <= NUM_STEPS; ++i) {
        // do stuff...
        command->setExecutionInfo(SiLA2::CReal{i / NUM_STEPS});
    }

    auto Response = Retreat_Responses{};
    // TODO: Fill the response fields
    return Response;
}

ApproachTo_Responses CRobotControllerImpl::ApproachTo(ApproachToWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;
    // TODO: Validate request parameters...

    // TODO: Write actual Command implementation logic...
    const double NUM_STEPS = 10.0;
    for (int i = 0; i <= NUM_STEPS; ++i) {
        // do stuff...
        command->setExecutionInfo(SiLA2::CReal{i / NUM_STEPS});
    }

    auto Response = ApproachTo_Responses{};
    // TODO: Fill the response fields
    return Response;
}

TransportPlate_Responses CRobotControllerImpl::TransportPlate(TransportPlateWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;
    // TODO: Validate request parameters...

    // TODO: Write actual Command implementation logic...
    const double NUM_STEPS = 10.0;
    for (int i = 0; i <= NUM_STEPS; ++i) {
        // do stuff...
        command->setExecutionInfo(SiLA2::CReal{i / NUM_STEPS});
    }

    auto Response = TransportPlate_Responses{};
    // TODO: Fill the response fields
    return Response;
}

PickPlate_Responses CRobotControllerImpl::PickPlate(PickPlateWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;
    // TODO: Validate request parameters...

    // TODO: Write actual Command implementation logic...

    auto Response = PickPlate_Responses{};
    // TODO: Fill the response fields
    return Response;
}

PlacePlate_Responses CRobotControllerImpl::PlacePlate(PlacePlateWrapper* command) {
    const auto Request = command->parameters();
    qDebug() << "Request contains:" << Request;
    // TODO: Validate request parameters...

    // TODO: Write actual Command implementation logic...

    auto Response = PlacePlate_Responses{};
    // TODO: Fill the response fields
    return Response;
}

CheckOccupied_Responses CRobotControllerImpl::CheckOccupied(CheckOccupiedWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;
    // TODO: Validate request parameters...

    // TODO: Write actual Command implementation logic...

    auto Response = CheckOccupied_Responses{};
    // TODO: Fill the response fields
    return Response;
}

FollowPath_Responses CRobotControllerImpl::FollowPath(FollowPathWrapper* command) {
    const auto Request = command->parameters();
    qDebug() << "Request contains:" << Request;
    // TODO: Validate request parameters...

    // TODO: Write actual Command implementation logic...

    auto Response = FollowPath_Responses{};
    // TODO: Fill the response fields
    return Response;
}

/**
 * Sets all joints of the robot to the requested values. No checks or path planning is done.
 * 
 * @param command The SiLA command.
 * @return A empty response.
 */
SetToFrame_Responses CRobotControllerImpl::SetToFrame(SetToFrameWrapper* command) {
    const auto& request = command->parameters();
    qDebug() << "Request contains:" << request;
    const auto& frame = request.frame().frame();
    std::vector<double> jointValues(MAX_JOINTS);
    for (size_t i = 0; i < jointValues.size(); i++) {
        jointValues[i] = frame.at(i).value();
    }

    m_MoveGroup.setJointValueTarget(jointValues);
    const MoveItErrorCode err = m_MoveGroup.move();
    if (err != MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to call service SetToFrame");
        // TODO: throw a defined SiLA execution exception
        return {};
    }

    return SetToFrame_Responses{};
}

FollowFrames_Responses CRobotControllerImpl::FollowFrames(FollowFramesWrapper* command) {
    const auto request = command->parameters();
    qDebug() << "Request contains:" << request;
    // TODO: Validate request parameters...

    // TODO: Write actual Command implementation logic...

    auto response = FollowFrames_Responses{};
    // TODO: Fill the response fields
    return response;
}
