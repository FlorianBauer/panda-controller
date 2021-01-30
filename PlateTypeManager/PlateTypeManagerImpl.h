//============================================================================
/// \file    PlateTypeManagerImpl.h
/// \authors Florian Bauer <florian.bauer.dev@gmail.com>
/// \date    2021-01-26
/// \brief   Declaration of the CPlateTypeManagerImpl class
/// \note    Code generated by sila2codegenerator 0.3.3-dev
//============================================================================
#ifndef PLATETYPEMANAGERIMPL_H
#define PLATETYPEMANAGERIMPL_H

#include <map>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <sila_cpp/server/SiLAFeature.h>
#include <sila_cpp/data_types.h>
#include <sila_cpp/server/command/UnobservableCommand.h>
#include <sila_cpp/server/property/UnobservableProperty.h>
#include "Plate.h"
#include "PlateTypeManager.grpc.pb.h"

static const SiLA2::CDefinedExecutionError ERROR_PLATE_TYPE_ID_NOT_FOUND
{
    "PlateTypeIdNotFound",
    "The given plate type ID could not be found or does not exist."
};

/**
 * @brief The CPlateTypeManagerImpl class implements the PlateTypeManager feature
 *
 * @details Manager to define and remove plate types, which are required by the robot
 * gripper.
 */
class CPlateTypeManagerImpl final : public SiLA2::CSiLAFeature<sila2::de::fau::robot::platetypemanager::v1::PlateTypeManager> {
    // Using declarations for the Feature's Commands and Properties
    using GetPlateTypeCommand =
            SiLA2::CUnobservableCommandManager<&CPlateTypeManagerImpl::RequestGetPlateType>;
    using GetPlateTypeWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::platetypemanager::v1::GetPlateType_Parameters,
            sila2::de::fau::robot::platetypemanager::v1::GetPlateType_Responses>;
    using SetPlateTypeCommand =
            SiLA2::CUnobservableCommandManager<&CPlateTypeManagerImpl::RequestSetPlateType>;
    using SetPlateTypeWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::platetypemanager::v1::SetPlateType_Parameters,
            sila2::de::fau::robot::platetypemanager::v1::SetPlateType_Responses>;
    using DeletePlateTypeCommand =
            SiLA2::CUnobservableCommandManager<&CPlateTypeManagerImpl::RequestDeletePlateType>;
    using DeletePlateTypeWrapper = SiLA2::CUnobservableCommandWrapper<
            sila2::de::fau::robot::platetypemanager::v1::DeletePlateType_Parameters,
            sila2::de::fau::robot::platetypemanager::v1::DeletePlateType_Responses>;
    using PlateTypesProperty = SiLA2::CUnobservablePropertyWrapper<
            std::vector<SiLA2::CString>, &CPlateTypeManagerImpl::RequestGet_PlateTypes>;

public:
    /**
     * @brief C'tor
     *
     * @param parent The SiLA server instance that contains this Feature
     */
    explicit CPlateTypeManagerImpl(SiLA2::CSiLAServer* parent);

    /**
     * @brief GetPlateType Command
     *
     * @details Gets the plate type.
     *
     * @param Command The current GetPlateType Command Execution Wrapper
     * It contains the following Parameters:
     * @li PlateId The plate type to query.
     *
     * @return GetPlateType_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::platetypemanager::v1::GetPlateType_Responses GetPlateType(GetPlateTypeWrapper* command);

    /**
     * @brief SetPlateType Command
     *
     * @details Defines a plate type.
     *
     * @param Command The current SetPlateType Command Execution Wrapper
     * It contains the following Parameters:
     * @li DimX The X dimension of the plate.
     * @li DimY The Y dimension of the plate.
     * @li DimZ The Z dimension or height of the plate.
     * @li OffX The X offset for grabbing the plate.
     * @li OffY The Y offset for grabbing the plate.
     * @li OffZ The Z offset for grabbing the plate.
     *
     * @return SetPlateType_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::platetypemanager::v1::SetPlateType_Responses SetPlateType(SetPlateTypeWrapper* command);

    /**
     * @brief DeletePlateType Command
     *
     * @details Removes a plate from the manager.
     *
     * @param Command The current DeletePlateType Command Execution Wrapper
     * It contains the following Parameters:
     * @li PlateId The plate to delete.
     *
     * @return DeletePlateType_Responses The Command Response
     * It contains the following fields:
     * None
     *
     * @throw Validation Error if the given Parameter(s) are invalid
     */
    sila2::de::fau::robot::platetypemanager::v1::DeletePlateType_Responses DeletePlateType(DeletePlateTypeWrapper* command);

    bool hasPlateTypeId(const std::string& plateTypeId) const;
    Plate createPlate(const std::string& plateTypeId) const;
    std::shared_ptr<Plate> createSharedPlate(const std::string& plateTypeId) const;

private:
    GetPlateTypeCommand m_GetPlateTypeCommand;
    SetPlateTypeCommand m_SetPlateTypeCommand;
    DeletePlateTypeCommand m_DeletePlateTypeCommand;
    PlateTypesProperty m_PlateTypesProperty;
    std::map<std::string, nlohmann::json> m_JsonLabwares;
    const std::filesystem::path m_LabwareDir;

    static std::map<std::string, nlohmann::json> loadLabwareFilesToMap();
};

#endif  // PLATETYPEMANAGERIMPL_H