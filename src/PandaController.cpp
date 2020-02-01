#include <cstdlib>
#include <iostream>

#include "Config.h"

/**
 * Determines whether the byte-order is big- or little-endian.
 * The endianness itself is thereby dependent on the CPU architecture.
 * 
 * @return `true` if the byte-order is big-endian, otherwise `false`.
 */
static bool isBigEndian() {
    union {
        uint32_t i;
        char c[4];
    } bint = {0x01020304};

    return bint.c[0] == 1;
}

/**
 * Prints some information about the current build.
 */
static void printBuildInfo() {
    std::cout << EXECUTABLE_NAME << "\n"
            << "Version: "
            << VERSION_MAJOR << "."
            << VERSION_MINOR << "."
            << VERSION_PATCH << "\n"
            << "Build Type: " << BUILD_TYPE << "\n"
            << "Build Timestamp: " << BUILD_TIMESTAMP << "\n"
            << "Byte Order: " << (isBigEndian() ? "big" : "little") << "-endian\n"
            << "Compiler: " << COMPILER_ID << " " << COMPILER_VERSION << "\n";
}

int main(int argc, char* argv[]) {
    std::cout << "Hello World\n";
    printBuildInfo();
    return 0;
}
