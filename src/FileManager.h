#ifndef FILEMANAGER_H
#define FILEMANAGER_H

#include <string>
#include <filesystem>
#include <vector>
#include <nlohmann/json.hpp>

static constexpr char APP_DIR[] = ".sila/panda_controller";
static constexpr char SITES_DIR[] = "sites";
static constexpr char PLATE_TYPES_DIR[] = "plate_types";
static constexpr char SCENE_OBJECTS_DIR[] = "scene_objects";
static constexpr char COLLISION_OBJECTS_DIR[] = "collision_objects";
static constexpr char JSON_FILE_EXT[] = ".json";

class FileManager {
public:
    static std::filesystem::path getAppDir();
    static void checkAndCreateDir(const std::filesystem::path& dirPath);
    static std::vector<std::filesystem::path> collectJsonFilesFromDir(const std::filesystem::path& dirPath);
    static bool saveJsonToFile(const nlohmann::json& jsonStruct, const std::filesystem::path & filePath);
    static nlohmann::json loadJsonFromFile(const std::filesystem::path & filePath);

    FileManager();
    FileManager(const FileManager & orig);
    virtual ~FileManager();

private:

};

#endif /* FILEMANAGER_H */
