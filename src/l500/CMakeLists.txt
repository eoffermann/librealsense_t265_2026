# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
target_sources(${LRS_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/l500-depth.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/l500-private.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/l500-color.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/l500-device.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/l500-motion.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/l500-factory.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/l500-fw-update-device.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/l500-serializable.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/l500-options.cpp"

        "${CMAKE_CURRENT_LIST_DIR}/l500-depth.h"
        "${CMAKE_CURRENT_LIST_DIR}/l500-private.h"
        "${CMAKE_CURRENT_LIST_DIR}/l500-color.h"
        "${CMAKE_CURRENT_LIST_DIR}/l500-device.h"
        "${CMAKE_CURRENT_LIST_DIR}/l500-motion.h"
        "${CMAKE_CURRENT_LIST_DIR}/l500-factory.h"
        "${CMAKE_CURRENT_LIST_DIR}/l500-fw-update-device.h"
        "${CMAKE_CURRENT_LIST_DIR}/l500-serializable.h"
        "${CMAKE_CURRENT_LIST_DIR}/l500-options.h"
)

# get-mfr-ww lives under common/ but is used by the L500 device layer (manufacturing
# work-week decoding). Compiled in here so realsense2 is self-contained.
target_sources(${LRS_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/../../common/utilities/time/l500/get-mfr-ww.h"
        "${CMAKE_CURRENT_LIST_DIR}/../../common/utilities/time/l500/get-mfr-ww.cpp"
)
