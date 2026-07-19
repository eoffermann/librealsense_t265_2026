# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019-2026 RealSense, Inc. All Rights Reserved.

# Only files already ported to the current SDK architecture are compiled.
#
# tm-device.cpp/.h, tm-boot.h, t265-messages.h and message-print.h are present in the tree
# but deliberately left out of the build until Phase 3 ports tm2_device onto the current
# device and sensor base classes. Adding them before then only produces noise.
#
# The original version of this file also linked the `fw` target and required
# IMPORT_DEPTH_CAM_FW, because T265 is a FW-less device that is sent its firmware on every
# power-up. Upstream has since removed firmware bundling entirely, so firmware delivery is
# redesigned in Phase 4 rather than restored here.
target_sources(${LRS_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/tm-info.h"
        "${CMAKE_CURRENT_LIST_DIR}/tm-info.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/tm-boot.h"
        "${CMAKE_CURRENT_LIST_DIR}/tm-boot.cpp"
)
