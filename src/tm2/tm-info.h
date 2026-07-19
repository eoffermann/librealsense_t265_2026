// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017-2026 RealSense, Inc. All Rights Reserved.

#pragma once

#include <src/platform/platform-device-info.h>

#include <memory>
#include <vector>

namespace librealsense
{
    class context;

    // device_info for the T265 (TM2) tracking camera.
    //
    // T265 is a raw-USB device: it exposes no UVC and no HID nodes, so its
    // backend_device_group carries a single usb_device_info and nothing else.
    //
    class tm2_info : public platform::platform_device_info
    {
        typedef platform::platform_device_info super;

    public:
        explicit tm2_info( std::shared_ptr< context > const & ctx, platform::usb_device_info const & hwm )
            : platform_device_info( ctx, { { hwm } } )
        {
        }

        std::shared_ptr< device_interface > create_device() override;

        // platform_device_info::is_same_as() compares groups with
        // backend_device_group::operator==, which deliberately looks at only the uvc and hid
        // lists. For a USB-only device that would make every T265 compare equal to every
        // other T265 -- and to every recovery device -- so the USB nodes are compared here
        // explicitly instead.
        //
        bool is_same_as( std::shared_ptr< const device_info > const & other ) const override;

        // Picks booted T265 devices out of a USB device list.
        //
        // An unbooted T265 enumerates as a Movidius device (VID 0x03E7, PID 0x2150) and is
        // deliberately not picked up here: it must be sent its firmware first, after which it
        // re-enumerates with the PID matched below. See tm-boot.h.
        //
        static std::vector< std::shared_ptr< tm2_info > >
            pick_tm2_devices( std::shared_ptr< context > ctx,
                              std::vector< platform::usb_device_info > & usb );
    };
}
