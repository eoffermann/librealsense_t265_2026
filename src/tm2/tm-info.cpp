// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017-2026 RealSense, Inc. All Rights Reserved.

#include "tm-info.h"
#include "tm-device.h"

#include <src/context.h>
#include <src/platform/platform-utils.h>

#include <rsutils/easylogging/easyloggingpp.h>

namespace librealsense
{
    // PID of a booted T265. Before it is sent its firmware the device enumerates as a
    // Movidius bootloader (0x03E7:0x2150) instead -- see tm-boot.h.
    static const uint16_t T265_BOOTED_PID = 0x0B37;


    std::shared_ptr< device_interface > tm2_info::create_device()
    {
        return std::make_shared< tm2_device >(
            std::dynamic_pointer_cast< const tm2_info >( shared_from_this() ) );
    }


    bool tm2_info::is_same_as( std::shared_ptr< const device_info > const & other ) const
    {
        auto rhs = std::dynamic_pointer_cast< const tm2_info >( other );
        if( ! rhs )
            return false;

        // Compare the USB nodes directly; see the note in the header for why the inherited
        // implementation cannot be used.
        return ! list_changed( get_group().usb_devices, rhs->get_group().usb_devices );
    }


    std::vector< std::shared_ptr< tm2_info > >
    tm2_info::pick_tm2_devices( std::shared_ptr< context > ctx,
                                std::vector< platform::usb_device_info > & usb )
    {
        std::vector< std::shared_ptr< tm2_info > > results;

        // Deliberately does not talk to the hardware: this is also called with a stale list
        // when devices disconnect, at which point the device may already be gone.
        auto correct_pid = filter_by_product( usb, { T265_BOOTED_PID } );
        for( auto & dev : correct_pid )
            results.push_back( std::make_shared< tm2_info >( ctx, dev ) );

        if( ! results.empty() )
            LOG_INFO( "Picked " << results.size() << "/" << usb.size() << " T265 devices" );

        return results;
    }
}
