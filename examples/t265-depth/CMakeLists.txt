# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
cmake_minimum_required(VERSION 3.10)

project(RealsenseExamplesT265Depth)

# Needs BUILD_WITH_TM2 for the camera itself, and graphical examples for the point cloud view.
if(BUILD_GRAPHICAL_EXAMPLES AND BUILD_WITH_TM2)
    add_executable(rs-t265-depth
        rs-t265-depth.cpp
        ../example.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../../src/proc/t265-stereo-rectify.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../../src/proc/t265-stereo-match.cpp
        ${IMGUI_SOURCES})
    set_property(TARGET rs-t265-depth PROPERTY CXX_STANDARD 14)
    target_link_libraries(rs-t265-depth ${DEPENDENCIES})
    include_directories(../ ${CMAKE_CURRENT_LIST_DIR}/../../)
    set_target_properties (rs-t265-depth PROPERTIES FOLDER Examples)
    install(TARGETS rs-t265-depth RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
