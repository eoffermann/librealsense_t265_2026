# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
#
# T265 (TM2) firmware acquisition.
#
# T265 is a FW-less device: it holds no firmware of its own and must be sent an image over
# USB on every power-up. Upstream removed the firmware-bundling machinery that used to embed
# these images into the binary (PR #15063), and this fork does not resurrect it -- that
# subsystem would conflict on every merge from upstream.
#
# Instead the image is downloaded once at configure time, hash-verified, and left on disk.
# The path is compiled in as a default; RS2_T265_FW_PATH overrides it at runtime.

set(T265_FW_VERSION "0.2.0.951")
set(T265_FW_SHA1 "c3940ccbb0e3045603e4aceaa2d73427f96e24bc")
set(T265_FW_URL_BASE "https://librealsense.intel.com/Releases/TM2/FW/target"
    CACHE STRING "Base URL to download T265 firmware from")

set(T265_FW_DIR "${CMAKE_BINARY_DIR}/t265-firmware")
set(T265_FW_FILE "${T265_FW_DIR}/target-${T265_FW_VERSION}.mvcmd")

if(EXISTS "${T265_FW_FILE}")
    file(SHA1 "${T265_FW_FILE}" _existing_sha1)
else()
    set(_existing_sha1 "")
endif()

if(NOT _existing_sha1 STREQUAL T265_FW_SHA1)
    message(STATUS "Downloading T265 firmware ${T265_FW_VERSION}")
    file(DOWNLOAD
         "${T265_FW_URL_BASE}/${T265_FW_VERSION}/target-${T265_FW_VERSION}.mvcmd"
         "${T265_FW_FILE}"
         EXPECTED_HASH SHA1=${T265_FW_SHA1}
         STATUS _dl_status)
    list(GET _dl_status 0 _dl_code)
    if(NOT _dl_code EQUAL 0)
        # A warning rather than an error: the device is EOL and the URL may eventually go
        # away, but everything except booting an unbooted camera still builds and works.
        # A user with their own copy of the image can point RS2_T265_FW_PATH at it.
        message(WARNING
            "Failed to download T265 firmware (${_dl_status}).\n"
            "The build will continue, but an unbooted T265 cannot be started unless "
            "RS2_T265_FW_PATH points at a target-${T265_FW_VERSION}.mvcmd image.")
    else()
        message(STATUS "T265 firmware verified: ${T265_FW_FILE}")
    endif()
else()
    message(STATUS "T265 firmware already present and verified: ${T265_FW_FILE}")
endif()

# Install alongside the library so an installed build can still boot a device.
if(EXISTS "${T265_FW_FILE}")
    install(FILES "${T265_FW_FILE}" DESTINATION ${CMAKE_INSTALL_DATADIR}/librealsense2/firmware)
endif()
