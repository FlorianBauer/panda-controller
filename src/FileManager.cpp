#include <algorithm>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <cctype>
#include "FileManager.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

/**
 * Gets the absolute path to application directory where all the files are stored (e.g.
 * "/home/username/.sila/panda_controller"). This function works only on *nix systems and the 
 * actual existence of the path is not checked.
 * 
 * @return The absolute app directory as string.
 */
fs::path FileManager::getAppDir() {
    const char* homeDir;
    if ((homeDir = getenv("HOME")) == nullptr) {
        homeDir = getpwuid(getuid())->pw_dir;
    }
    return fs::path(std::string(homeDir)) / APP_DIR;
}

/**
 * Creates the directory if not yet existing.
 */
void FileManager::checkAndCreateDir(const fs::path& dirPath) {
    if (!fs::exists(dirPath)) {
        fs::create_directories(dirPath);
    }
}

/**
 * Queries all regular files with an *.json extension from the given directory and returns their
 * paths within a vector.
 * 
 * @param dirPath The directory path to collect the *.json files from.
 * @return A vector filled with the paths to the files.
 */
std::vector<fs::path> FileManager::collectJsonFilesFromDir(const fs::path& dirPath) {
    std::vector<fs::path> files;
    if (!fs::is_directory(dirPath) || fs::is_empty(dirPath)) {
        // nothing to collect
        return files;
    }

    for (const auto& entry : fs::directory_iterator(dirPath)) {
        if (entry.is_regular_file()) {
            // check for the ".json" file extension
            if (entry.path().extension().string().compare(JSON_FILE_EXT) == 0) {
                files.push_back(entry);
            }
        }
    }
    return files;
}

FileManager::FileManager() {

}

FileManager::FileManager(const FileManager& orig) {
}

FileManager::~FileManager() {
}

/**
 * Writes the given JSON struct into the given file path. Already existing files get overwritten.
 * 
 * @param jsonStruct The JSON struct to store.
 * @param filePath The file to write to.
 * @return true on success, otherwise false.
 */
bool FileManager::saveJsonToFile(const json& jsonStruct, const fs::path& filePath) {
    if (fs::exists(filePath)) {
        if (!fs::is_regular_file(filePath)) {
            // invalid path.
            return false;
        }
    }
    // write prettified JSON to file
    std::ofstream outStream(filePath);
    outStream << std::setw(4) << jsonStruct << std::endl;
    outStream.close();
    return true;
}

json FileManager::loadJsonFromFile(const fs::path& filePath) {
    json jsonStruct;
    std::ifstream jsonStream(filePath);
    jsonStream >> jsonStruct;
    jsonStream.close();
    return jsonStruct;
}
