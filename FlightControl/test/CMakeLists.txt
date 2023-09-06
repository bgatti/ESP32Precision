# This is the project CMakeLists.txt file for the test subproject
cmake_minimum_required(VERSION 3.16)

# Include the components directory of the main application:
#
#set(EXTRA_COMPONENT_DIRS "../components" "../managed_components/.")

include(FetchContent)
FetchContent_Declare(
  espidflib
  GIT_REPOSITORY https://github.com/UncleRus/esp-idf-lib.git
)
FetchContent_MakeAvailable(espidflib)
set(EXTRA_COMPONENT_DIRS ${espidflib_SOURCE_DIR}/components  "../components" "../managed_components/.")


# Set the components to include the tests for.
# This can be overriden from CMake cache:
# - when invoking CMake directly: cmake -D TEST_COMPONENTS="xxxxx" ..
# - when using idf.py: idf.py -T xxxxx build
#
set(TEST_COMPONENTS "Guidance" "IMU" "LIS2DW12" "MotorDriver" CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(flightcontrol_unit_test)
