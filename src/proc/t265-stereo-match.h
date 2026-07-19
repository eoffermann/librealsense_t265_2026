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
    int   window        = 11;    // aggregation window, odd
    float uniqueness    = 0.97f; // best cost must beat second-best by this ratio
    bool  lr_check      = true;  // reject matches that disagree left-to-right
    int   lr_max_diff   = 2;     // tolerance for that check, in pixels
    int   min_variance  = 15;    // reject textureless windows outright
};

// Defaults above were chosen by sweeping one captured frame pair. Measured coverage:
//
//   strict (window 7, uniq 0.85, var 100, lr 1)   17%   352ms
//   uniqueness 0.95 alone                         29%   350ms
//   these defaults                                45%   515ms
//   these defaults with lr_check off              51%   519ms
//
// Uniqueness dominates -- it alone takes 17% to 29%. Disabling the left-right check buys
// only a further 6%, so it stays on: it is the main defence against false matches in
// repetitive texture and cheap now that it reuses the main pass.
//
// Median depth held at 1.10-1.22m across every configuration against a scene about 1.2m
// away, which is the reason to trust the looser settings: they admit more real geometry
// rather than scattering noise at random depths.
//
// Passive stereo still cannot invent texture. Blank walls will stay empty whatever these
// are set to; the honest fix for that is a smarter algorithm, not looser thresholds.


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
