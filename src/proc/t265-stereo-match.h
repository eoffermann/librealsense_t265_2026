// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#pragma once

#include <cstdint>
#include <vector>

namespace librealsense {
namespace t265_stereo {


// Block-matching stereo for the rectified T265 fisheye pair.
//
// Deliberately a census-transform matcher rather than plain SAD on intensities. The two
// fisheye cameras run their own exposure and have different vignetting, so absolute intensity
// differs between them; census compares each pixel only to its own neighbourhood, so it cares
// about local structure rather than absolute brightness and tolerates that mismatch.
//
// Cost aggregation uses a sliding-window sum, which keeps the whole thing O(W*H*D) rather than
// O(W*H*D*window^2). At 412x300 with 112 disparities that is about 14M operations per frame --
// slow by hardware-stereo standards, but entirely usable on a host CPU.


struct matcher_config
{
    int   max_disparity = 112;   // search range, matched to rectified_config
    int   window        = 7;     // aggregation window, odd
    float uniqueness    = 0.85f; // best cost must beat second-best by this ratio
    bool  lr_check      = true;  // reject matches that disagree left-to-right
    int   lr_max_diff   = 1;     // tolerance for that check, in pixels
    int   min_variance  = 100;   // reject textureless windows outright
};


// Computes disparity for the left image against the right.
//
// Both inputs are 8-bit rectified images of the same size. Output is one float per pixel:
// disparity in pixels (sub-pixel refined), or a negative value where no confident match
// exists. Invalid pixels are common and expected -- passive stereo cannot invent texture.
//
void match_disparity( uint8_t const * left,
                      uint8_t const * right,
                      int width, int height,
                      matcher_config const & cfg,
                      float * disparity_out );


// Converts disparity to depth in metres:  Z = focal * baseline / disparity.
//
// `baseline_m` is the magnitude of the stereo baseline (0.0639 m on this device) and
// `focal_px` the rectified focal length. Pixels with invalid disparity, or outside
// [min_z, max_z], become 0 -- the same convention librealsense uses for "no depth here".
//
void disparity_to_depth_z16( float const * disparity,
                             int width, int height,
                             float focal_px, float baseline_m,
                             float depth_units,
                             float min_z, float max_z,
                             uint16_t * depth_out );


}  // namespace t265_stereo
}  // namespace librealsense
