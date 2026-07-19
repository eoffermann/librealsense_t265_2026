// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#pragma once

#include <librealsense2/h/rs_types.h>

#include <cstdint>
#include <vector>

namespace librealsense {
namespace t265_stereo {


// Fisheye stereo rectification for the T265.
//
// T265 is not a depth camera, but it carries two global-shutter fisheye cameras in a stereo
// pair with factory calibration, so a depth map can be computed on the host. The lenses are
// very wide (~163 degrees) and modelled with Kannala-Brandt 4-parameter distortion, so the
// images must be undistorted and rectified onto a common pinhole frame before any stereo
// matching can work.
//
// Only a central crop is rectified. Recovering the full fisheye field of view would need an
// enormous output image for very little useful disparity at the edges, and depth at the
// periphery of a 163-degree lens is not worth the pixels.
//
// The approach follows Intel's own t265_stereo.py reference: keep the left camera's frame as
// the rectified frame, rotate the right camera into it, and shift the right projection by
// baseline * focal. That simplification is valid here because the measured extrinsics put the
// baseline essentially along x (translation y and z are ~0) -- verified on hardware at
// [-0.064, -0.000, -0.000], a 63.9 mm baseline.


struct rectified_config
{
    // Output geometry. The matcher needs `max_disparity` extra columns on the left so that
    // every pixel in the square region of interest has somewhere to search.
    int   height_px     = 300;
    float fov_rad       = 1.5707963f;  // 90 degrees
    int   max_disparity = 112;         // must match the matcher

    int width_px() const { return height_px + max_disparity; }

    // Focal length implied by the requested output height and field of view:
    //
    //         h
    //   ---------------
    //    \     |     /
    //      \   | f  /
    //        \ fov /
    //          \|/
    float focal_px() const;

    // Principal point. x is offset by max_disparity so the region of interest stays centred
    // once the search margin is cropped away.
    float cx() const { return ( height_px - 1 ) * 0.5f + max_disparity; }
    float cy() const { return ( height_px - 1 ) * 0.5f; }
};


// A per-pixel remap table: for each output pixel, where to sample in the source image.
// Precomputed once per calibration, then applied to every frame.
struct remap_table
{
    int width = 0;
    int height = 0;
    std::vector< float > src_x;  // size width*height; negative means "no source pixel"
    std::vector< float > src_y;

    bool valid() const { return width > 0 && height > 0 && src_x.size() == size_t( width ) * height; }
};


// Builds the remap table for one camera of the pair.
//
// `intr` is that camera's fisheye intrinsics (expects RS2_DISTORTION_KANNALA_BRANDT4).
// `rotation` is the 3x3 row-major rotation taking a ray in the rectified frame into that
// camera's frame -- identity for the left camera, and the left-to-right rotation for the
// right one.
//
remap_table build_remap( rs2_intrinsics const & intr,
                         float const rotation[9],
                         rectified_config const & cfg );


// Applies a remap table to an 8-bit single-channel image, bilinearly.
// Pixels with no valid source are written as 0.
void remap_y8( uint8_t const * src, int src_w, int src_h,
               remap_table const & table,
               uint8_t * dst );


// The intrinsics of the rectified output, for downstream consumers (depth frames,
// point clouds). The rectified image is a pinhole projection, so it carries no distortion.
rs2_intrinsics rectified_intrinsics( rectified_config const & cfg );


}  // namespace t265_stereo
}  // namespace librealsense
