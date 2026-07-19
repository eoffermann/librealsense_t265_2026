// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2026 RealSense, Inc. All Rights Reserved.
#pragma once

#include <src/platform/platform-device-info.h>

#include "l500-private.h"

namespace librealsense
{
    // Rewritten against the current device_info architecture, mirroring d400_info.
    //
    // The old shape -- create( ctx, register_device_notifications ) plus a get_device_data()
    // that rebuilt the group on demand, with _depth / _hwm / _hid stored here -- no longer
    // exists. The group now lives in platform_device_info and creation takes no arguments.
    //
    // Note the hardware-monitor member became a vector to match the group's shape; the old
    // code stored a single usb_device_info and wrapped it inline.
    //
    class l500_info : public platform::platform_device_info
    {
    public:
        std::shared_ptr< device_interface > create_device() override;

        l500_info( std::shared_ptr< context > const & ctx,
                   std::vector< platform::uvc_device_info > && depth,
                   std::vector< platform::usb_device_info > && hwm,
                   std::vector< platform::hid_device_info > && hid )
            : platform_device_info( ctx, { std::move( depth ), std::move( hwm ), std::move( hid ) } )
        {
        }

        static std::vector< std::shared_ptr< l500_info > >
            pick_l500_devices( std::shared_ptr< context > ctx,
                               platform::backend_device_group & group );
    };
}
