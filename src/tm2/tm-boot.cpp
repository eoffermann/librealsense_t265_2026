// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017-2026 RealSense, Inc. All Rights Reserved.

#include "tm-boot.h"

#include <src/usb/usb-device.h>
#include <src/usb/usb-enumerator.h>

#include <rsutils/easylogging/easyloggingpp.h>

#include <cstdlib>
#include <fstream>

namespace librealsense {
namespace platform {

namespace {

    const uint16_t MOVIDIUS_BOOTLOADER_VID = 0x03E7;
    const uint16_t MOVIDIUS_BOOTLOADER_PID = 0x2150;

    // The firmware image is roughly 9 MB, sent as a single bulk transfer. The original
    // implementation allowed 1 second, which is tight even on a healthy link; a generous
    // timeout means a slow-but-working transfer is not misreported as a failure.
    const uint32_t BOOT_TRANSFER_TIMEOUT_MS = 15000;

    const char * FW_PATH_ENV_VAR = "RS2_T265_FW_PATH";


    std::vector< uint8_t > load_target_firmware()
    {
        // RS2_T265_FW_PATH wins if set; otherwise fall back to the image CMake downloaded
        // and hash-verified at configure time.
        char const * path = getenv( FW_PATH_ENV_VAR );
#ifdef T265_DEFAULT_FW_PATH
        if( ! path )
            path = T265_DEFAULT_FW_PATH;
#endif
        if( ! path )
        {
            LOG_ERROR( "Cannot boot T265: no firmware image available. Set " << FW_PATH_ENV_VAR
                       << " to a target-*.mvcmd image, or rebuild with network access so the "
                          "image can be downloaded." );
            return {};
        }

        std::ifstream file( path, std::ios::binary | std::ios::ate );
        if( ! file )
        {
            LOG_ERROR( "Cannot boot T265: failed to open firmware image at " << path );
            return {};
        }

        auto const size = static_cast< size_t >( file.tellg() );
        file.seekg( 0, std::ios::beg );

        std::vector< uint8_t > fw( size );
        if( ! file.read( reinterpret_cast< char * >( fw.data() ), size ) )
        {
            LOG_ERROR( "Cannot boot T265: failed to read firmware image at " << path );
            return {};
        }

        LOG_INFO( "Loaded " << fw.size() << " bytes of T265 firmware from " << path );
        return fw;
    }

}  // namespace


bool tm_boot( const std::vector< usb_device_info > & devices )
{
    bool found = false;

    for( auto const & device_info : devices )
    {
        if( device_info.vid != MOVIDIUS_BOOTLOADER_VID || device_info.pid != MOVIDIUS_BOOTLOADER_PID )
            continue;

        LOG_INFO( "Found an unbooted T265 to boot" );
        found = true;

        auto fw = load_target_firmware();
        if( fw.empty() )
            continue;  // already logged

        auto dev = usb_enumerator::create_usb_device( device_info );
        if( ! dev )
        {
            LOG_ERROR( "Failed to create a USB device for the unbooted T265" );
            continue;
        }

        auto messenger = dev->open( 0 );
        if( ! messenger )
        {
            LOG_ERROR( "Failed to open the T265 zero interface" );
            continue;
        }

        auto iface = dev->get_interface( 0 );
        if( ! iface )
        {
            LOG_ERROR( "Failed to get interface 0 of the unbooted T265" );
            continue;
        }

        auto endpoint = iface->first_endpoint( RS2_USB_ENDPOINT_DIRECTION_WRITE );
        if( ! endpoint )
        {
            LOG_ERROR( "Failed to find a bulk write endpoint on the T265 bootloader" );
            continue;
        }

        uint32_t transferred = 0;
        auto const status = messenger->bulk_transfer( endpoint,
                                                      fw.data(),
                                                      static_cast< uint32_t >( fw.size() ),
                                                      transferred,
                                                      BOOT_TRANSFER_TIMEOUT_MS );

        // Report partial transfers distinctly from outright failures: a short write means the
        // link or the timeout is the problem, not the image or the endpoint.
        if( status != RS2_USB_STATUS_SUCCESS )
            LOG_ERROR( "Error booting T265: USB status " << status
                       << " after " << transferred << "/" << fw.size() << " bytes" );
        else if( transferred != fw.size() )
            LOG_ERROR( "Partial T265 firmware transfer: " << transferred << "/" << fw.size() << " bytes" );
        else
            LOG_INFO( "T265 firmware sent (" << transferred << " bytes); "
                      "the device should re-enumerate as 0x8087:0x0B37 shortly" );
    }

    return found;
}


}  // namespace platform
}  // namespace librealsense
