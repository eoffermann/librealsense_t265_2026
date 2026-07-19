// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// Host-side depth and point cloud from the T265's stereo fisheye pair.
//
// The T265 is a tracking camera, not a depth camera, and never shipped with depth support.
// But it carries two global-shutter fisheye cameras in a calibrated stereo pair, so a depth
// map can be computed on the host. Quality will not approach a D400 -- this is passive stereo
// with no projector and no hardware acceleration -- but it is real geometry.
//
// Only a 90-degree central crop is reconstructed. Recovering the full ~163-degree fisheye
// field would cost an enormous output image for very little usable disparity at the edges.
//
// The computed depth is republished through a software_device so that it is a genuine
// rs2::depth_frame. That means rs2::pointcloud does the reconstruction, and the result behaves
// like any other RealSense point cloud -- including export to PLY.

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

#include "example.hpp"

#include "../../src/proc/t265-stereo-rectify.h"
#include "../../src/proc/t265-stereo-match.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

using namespace librealsense::t265_stereo;

int main() try
{
    rs2::context ctx;

    rs2::pipeline pipe( ctx );
    rs2::config cfg_rs;
    cfg_rs.enable_stream( RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8 );
    cfg_rs.enable_stream( RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8 );
    auto profile = pipe.start( cfg_rs );

    auto lp = profile.get_stream( RS2_STREAM_FISHEYE, 1 ).as< rs2::video_stream_profile >();
    auto rp = profile.get_stream( RS2_STREAM_FISHEYE, 2 ).as< rs2::video_stream_profile >();
    auto Kl = lp.get_intrinsics();
    auto Kr = rp.get_intrinsics();
    auto ext = lp.get_extrinsics_to( rp );

    // rs2_extrinsics.rotation is column-major; the remap builder wants row-major.
    float R[9];
    for( int r = 0; r < 3; ++r )
        for( int c = 0; c < 3; ++c )
            R[r * 3 + c] = ext.rotation[c * 3 + r];
    float const identity[9] = { 1,0,0, 0,1,0, 0,0,1 };

    float const baseline = std::sqrt( ext.translation[0] * ext.translation[0]
                                    + ext.translation[1] * ext.translation[1]
                                    + ext.translation[2] * ext.translation[2] );

    rectified_config rcfg;
    matcher_config   mcfg;
    mcfg.max_disparity = rcfg.max_disparity;

    auto map_l = build_remap( Kl, identity, rcfg );
    auto map_r = build_remap( Kr, R, rcfg );

    int const W = rcfg.width_px();
    int const H = rcfg.height_px;
    float const focal = rcfg.focal_px();
    float const depth_units = 0.001f;   // 1 mm, as good a choice as any here

    std::cout << "T265 stereo depth\n"
              << "  rectified " << W << "x" << H << " (ROI " << ( W - rcfg.max_disparity )
              << "x" << H << "), focal " << focal << "px, baseline " << baseline << "m\n"
              << "  depth range " << ( focal * baseline / rcfg.max_disparity ) << "m .. "
              << ( focal * baseline / 2.f ) << "m\n\n";

    // Republish the computed depth as a real RealSense depth stream. The point cloud, and
    // anything else downstream, then works exactly as it would for a depth camera.
    rs2::software_device sw;
    auto sw_sensor = sw.add_sensor( "T265 Stereo Depth" );

    // Only the region of interest is published; the disparity search margin is cropped.
    int const roi_w = W - rcfg.max_disparity;
    rs2_intrinsics roi_intr = rectified_intrinsics( rcfg );
    roi_intr.width = roi_w;
    roi_intr.ppx  -= rcfg.max_disparity;   // principal point moves with the crop

    rs2_video_stream vs = {};
    vs.type = RS2_STREAM_DEPTH;
    vs.index = 0;
    vs.uid = 0;
    vs.width = roi_w;
    vs.height = H;
    vs.fps = 30;
    vs.bpp = 2;
    vs.fmt = RS2_FORMAT_Z16;
    vs.intrinsics = roi_intr;
    auto sw_profile = sw_sensor.add_video_stream( vs );

    // rs2::pointcloud needs the depth scale to turn Z16 into metres.
    sw_sensor.add_read_only_option( RS2_OPTION_DEPTH_UNITS, depth_units );

    rs2::syncer sw_sync;
    sw_sensor.open( sw_profile );
    sw_sensor.start( sw_sync );

    rs2::pointcloud pc;
    rs2::colorizer colorizer;
    window app( 1280, 720, "RealSense T265 Stereo Depth" );
    glfw_state app_state;
    register_glfw_callbacks( app, app_state );

    std::vector< uint8_t >  rect_l( size_t( W ) * H ), rect_r( size_t( W ) * H );
    std::vector< float >    disparity( size_t( W ) * H );
    std::vector< uint16_t > depth_full( size_t( W ) * H );
    std::vector< uint16_t > depth_roi( size_t( roi_w ) * H );

    int frame_number = 0;

    while( app )
    {
        rs2::frameset fs;
        if( ! pipe.try_wait_for_frames( &fs, 5000 ) )
            continue;

        auto fl = fs.get_fisheye_frame( 1 );
        auto fr = fs.get_fisheye_frame( 2 );
        if( ! fl || ! fr )
            continue;

        auto const t0 = std::chrono::steady_clock::now();

        remap_y8( (uint8_t const *)fl.get_data(), Kl.width, Kl.height, map_l, rect_l.data() );
        remap_y8( (uint8_t const *)fr.get_data(), Kr.width, Kr.height, map_r, rect_r.data() );

        match_disparity( rect_l.data(), rect_r.data(), W, H, mcfg, disparity.data() );

        disparity_to_depth_z16( disparity.data(), W, H, focal, baseline, depth_units,
                                0.15f, 8.f, depth_full.data() );

        // Crop away the disparity search margin.
        for( int y = 0; y < H; ++y )
            std::memcpy( depth_roi.data() + size_t( y ) * roi_w,
                         depth_full.data() + size_t( y ) * W + rcfg.max_disparity,
                         size_t( roi_w ) * sizeof( uint16_t ) );

        auto const t1 = std::chrono::steady_clock::now();

        rs2_software_video_frame swf = {};
        swf.pixels = depth_roi.data();
        swf.deleter = []( void * ) {};          // buffer outlives the frame; reused next pass
        swf.stride = roi_w * 2;
        swf.bpp = 2;
        swf.timestamp = fl.get_timestamp();
        swf.domain = RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK;
        swf.frame_number = frame_number++;
        swf.profile = sw_profile.get();
        sw_sensor.on_video_frame( swf );

        rs2::frameset sw_fs = sw_sync.wait_for_frames();
        auto depth = sw_fs.get_depth_frame();
        if( ! depth )
            continue;

        // Colorize first and map_to before calculate: draw_pointcloud renders via
        // points.get_texture_coordinates(), and those are only populated by map_to. Without
        // it every point samples an undefined texel and the cloud draws black.
        auto coloured = colorizer.process( depth );
        pc.map_to( coloured );
        auto points = pc.calculate( depth );
        app_state.tex.upload( coloured );

        draw_pointcloud( app.width(), app.height(), app_state, points );

        if( ( frame_number % 15 ) == 0 )
        {
            auto ms = std::chrono::duration< double, std::milli >( t1 - t0 ).count();
            size_t valid = 0;
            for( auto v : depth_roi ) if( v ) ++valid;
            std::cout << "frame " << frame_number << "  " << ms << "ms  coverage "
                      << ( 100.0 * valid / depth_roi.size() ) << "%  points "
                      << points.size() << "\r" << std::flush;
        }
    }

    pipe.stop();
    std::cout << "\n";
    return EXIT_SUCCESS;
}
catch( rs2::error const & e )
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args()
              << "):\n    " << e.what() << "\n";
    return EXIT_FAILURE;
}
catch( std::exception const & e )
{
    std::cerr << e.what() << "\n";
    return EXIT_FAILURE;
}
