// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "fw-update/fw-update-device.h"

namespace librealsense
{
    class l500_update_device : public update_device
    {
    public:
        static const uint16_t DFU_VERSION_MASK = 0xFE;
        static const uint16_t DFU_VERSION_VALUE = 0x4A; // On Units with old DFU payload can be 74/75 decimal

        // The L515 device EEPROM has different bytes order then D4xx device.
        // this struct overrides the generic serial_number_data struct at fw-update-device.h
        struct serial_number_data
        {
            uint8_t spare[2];
            uint8_t serial[6];
        };

        l500_update_device( std::shared_ptr< const device_info > const & dev_info,
                            std::shared_ptr< platform::usb_device > const & usb_device );
        virtual ~l500_update_device() = default;

        void update(const void* fw_image, int fw_image_size, rs2_update_progress_callback_sptr = nullptr) const override;
        virtual bool check_fw_compatibility(const std::vector<uint8_t>& image) const override;

    protected:
        // get_name/get_product_line/get_serial_number are non-virtual on the base now and
        // read its own members, so the overrides (and the shadowing members) are gone.
        // parse_serial_number stays: L515's EEPROM byte order differs.
        std::string parse_serial_number(const std::vector<uint8_t>& buffer) const override;
    };
}
