// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017-2026 RealSense, Inc. All Rights Reserved.

#pragma once

#include <src/usb/usb-types.h>

#include <vector>

namespace librealsense {
namespace platform {


// Sends firmware to any unbooted T265 found in `devices`.
//
// T265 is a FW-less device: it holds no firmware of its own and must be sent an image on
// every power-up. Until that happens it enumerates as a Movidius bootloader
// (VID 0x03E7, PID 0x2150). Once booted it re-enumerates as a T265 proper
// (VID 0x8087, PID 0x0B37), which is the PID tm2_info::pick_tm2_devices matches.
//
// Returns true if at least one unbooted device was found -- whether or not booting it
// succeeded -- to tell the caller the USB device list is about to change and is worth
// re-querying.
//
// The firmware image location is taken from the RS2_T265_FW_PATH environment variable.
// This is an interim mechanism: upstream removed firmware bundling entirely, so Phase 4
// of the restoration plan decides how the image is delivered properly.
//
bool tm_boot( const std::vector< usb_device_info > & devices );


}  // namespace platform
}  // namespace librealsense
