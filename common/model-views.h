// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 RealSense, Inc. All Rights Reserved.

#pragma once
#include <librealsense2/rs.hpp>

#include "rendering.h"
#include "ux-window.h"
#include "parser.hpp"
#include "rs-config.h"

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include "opengl3.h"
#include <imgui.h>
#include <realsense_imgui.h>
#include <imgui_impl_glfw.h>
#include <map>
#include <set>
#include <array>
#include <unordered_map>

#include "objects-in-frame.h"
#include "processing-block-model.h"

#include "realsense-ui-advanced-mode.h"
#include "fw-update-helper.h"
#include "updates-model.h"
#include "calibration-model.h"
#include <rsutils/time/periodic-timer.h>
#include "option-model.h"


namespace rs2
{
    void prepare_config_file();

    // Defined in model-views.cpp. Declared here so the T265 localization-map import/export
    // in device-model.cpp can reach them.
    std::vector<uint8_t> bytes_from_bin_file(const std::string& filename);
    void bin_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes);

    bool frame_metadata_to_csv( const std::string & filename, rs2::frame frame );

    bool motion_data_to_csv( const std::string & filename, rs2::frame frame );

    bool pose_data_to_csv( const std::string & filename, rs2::frame frame );

    void open_issue(std::string body);

    class option_model;

    void hyperlink(ux_window& window, const char* title, const char* link);

    static const float FEET_TO_METER = 0.3048f;

    template<class T>
    void sort_together(std::vector<T>& vec, std::vector<std::string>& names)
    {
        std::vector<std::pair<T, std::string>> pairs(vec.size());
        for (size_t i = 0; i < vec.size(); i++) pairs[i] = std::make_pair(vec[i], names[i]);

        std::sort(begin(pairs), end(pairs),
        [](const std::pair<T, std::string>& lhs,
           const std::pair<T, std::string>& rhs) {
            return lhs.first < rhs.first;
        });

        for (size_t i = 0; i < vec.size(); i++)
        {
            vec[i] = pairs[i].first;
            names[i] = pairs[i].second;
        }
    }

    template<class T>
    void push_back_if_not_exists(std::vector<T>& vec, T value)
    {
        auto it = std::find(vec.begin(), vec.end(), value);
        if (it == vec.end()) vec.push_back(value);
    }

    struct notification_model;
    typedef std::map<int, rect> streams_layout;

    std::vector<std::string> get_device_info(const device& dev, bool include_location = true);

    using color = std::array<float, 3>;
    using face = std::array<float3, 4>;
    using colored_cube = std::array<std::pair<face, color>, 6>;
    using tracked_point = std::pair<rs2_vector, unsigned int>; // translation and confidence

    class tm2_model
    {
    public:
        tm2_model() : _trajectory_tracking(true)
        {
        }
        void draw_trajectory(bool is_trajectory_button_pressed);
        void update_model_trajectory(const pose_frame& pose, bool track);
        void record_trajectory(bool on) { _trajectory_tracking = on; };
        void reset_trajectory() { trajectory.clear(); };

    private:
        void add_to_trajectory(tracked_point& p);

        const float len_x = 0.1f;
        const float len_y = 0.03f;
        const float len_z = 0.01f;
        /*
        4--------------------------3
        /|                         /|
        5-|------------------------6 |
        | /1                       | /2
        |/                         |/
        7--------------------------8
        */
        float3 v1{ -len_x / 2, -len_y / 2,  len_z / 2 };
        float3 v2{ len_x / 2, -len_y / 2,  len_z / 2 };
        float3 v3{ len_x / 2,  len_y / 2,  len_z / 2 };
        float3 v4{ -len_x / 2,  len_y / 2,  len_z / 2 };
        float3 v5{ -len_x / 2,  len_y / 2, -len_z / 2 };
        float3 v6{ len_x / 2,  len_y / 2, -len_z / 2 };
        float3 v7{ -len_x / 2, -len_y / 2, -len_z / 2 };
        float3 v8{ len_x / 2, -len_y / 2, -len_z / 2 };
        face f1{ { v1,v2,v3,v4 } }; //Back
        face f2{ { v2,v8,v6,v3 } }; //Right side
        face f3{ { v4,v3,v6,v5 } }; //Top side
        face f4{ { v1,v4,v5,v7 } }; //Left side
        face f5{ { v7,v8,v6,v5 } }; //Front
        face f6{ { v1,v2,v8,v7 } }; //Bottom side

        std::array<color, 6> colors{ {
            { { 0.5f, 0.5f, 0.5f } }, //Back
        { { 0.7f, 0.7f, 0.7f } }, //Right side
        { { 1.0f, 0.7f, 0.7f } }, //Top side
        { { 0.7f, 0.7f, 0.7f } }, //Left side
        { { 0.4f, 0.4f, 0.4f } }, //Front
        { { 0.7f, 0.7f, 0.7f } }  //Bottom side
            } };

        colored_cube camera_box{ { { f1,colors[0] },{ f2,colors[1] },{ f3,colors[2] },{ f4,colors[3] },{ f5,colors[4] },{ f6,colors[5] } } };

        std::vector<tracked_point> trajectory;
        std::vector<float2> boundary;
        bool                _trajectory_tracking;

    };

    class press_button_model
    {
    public:
        press_button_model(const char* icon_default, const char* icon_pressed, std::string tooltip_default, std::string tooltip_pressed,
                           bool init_pressed)
        {
            state_pressed = init_pressed;
            tooltip[unpressed] = tooltip_default;
            tooltip[pressed] = tooltip_pressed;
            icon[unpressed] = icon_default;
            icon[pressed] = icon_pressed;
        }

        void toggle_button() { state_pressed = !state_pressed; }
        void set_button_pressed(bool p) { state_pressed = p; }
        bool is_pressed() { return state_pressed; }
        std::string get_tooltip() { return(state_pressed ? tooltip[pressed] : tooltip[unpressed]); }
        std::string get_icon() { return(state_pressed ? icon[pressed] : icon[unpressed]); }

    private:
        enum button_state
        {
            unpressed, //default
            pressed
        };

        bool state_pressed = false;
        std::string tooltip[2];
        std::string icon[2];
    };

    bool yes_no_dialog(const std::string& title, const std::string& message_text, bool& approved, ux_window& window, const std::string& error_message, bool disabled = false, const std::string& disabled_reason = "");
    bool status_dialog(const std::string& title, const std::string& process_topic_text, const std::string& process_status_text, bool enable_close, ux_window& window);

    struct notifications_model;
    void export_frame(const std::string& fname, std::unique_ptr<rs2::filter> exporter, notifications_model& ns, rs2::frame data, bool notify = true);

    // Auxillary function to save stream data in its internal (raw) format
    bool save_frame_raw_data(const std::string& filename, rs2::frame frame);
}
