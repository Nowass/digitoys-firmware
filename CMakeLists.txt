# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
#idf_build_set_property(MINIMAL_BUILD ON)

set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components
    ${CMAKE_CURRENT_SOURCE_DIR}/components/digitoys-core
    ${CMAKE_CURRENT_SOURCE_DIR}/components/lidar-driver
    ${CMAKE_CURRENT_SOURCE_DIR}/components/adas-pwm-driver
)

project(digitoys-firmware)
