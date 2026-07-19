// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "t265-stereo-rectify.h"

#include <cmath>

namespace librealsense {
namespace t265_stereo {


float rectified_config::focal_px() const
{
    return ( height_px * 0.5f ) / std::tan( fov_rad * 0.5f );
}


rs2_intrinsics rectified_intrinsics( rectified_config const & cfg )
{
    rs2_intrinsics out = {};
    out.width  = cfg.width_px();
    out.height = cfg.height_px;
    out.fx     = cfg.focal_px();
    out.fy     = cfg.focal_px();
    out.ppx    = cfg.cx();
    out.ppy    = cfg.cy();
    // Rectified output is an ideal pinhole projection -- no distortion by construction.
    out.model  = RS2_DISTORTION_NONE;
    for( int i = 0; i < 5; ++i )
        out.coeffs[i] = 0.f;
    return out;
}


namespace {

// Kannala-Brandt 4-parameter forward projection: a 3D ray in camera coordinates to a pixel.
//
// theta is the angle from the optical axis; the distortion polynomial is applied to theta
// rather than to the radius, which is what lets this model cope with a field of view beyond
// 180 degrees where a Brown-Conrady model degenerates.
//
// Returns false for rays that cannot be projected (at or behind the optical centre).
bool project_kannala_brandt4( rs2_intrinsics const & intr,
                              float x, float y, float z,
                              float & u, float & v )
{
    float const r = std::sqrt( x * x + y * y );

    // A ray exactly on the optical axis lands on the principal point; theta/r is 0/0 there.
    if( r < 1e-9f )
    {
        if( z <= 0.f )
            return false;
        u = intr.ppx;
        v = intr.ppy;
        return true;
    }

    float const theta = std::atan2( r, z );

    float const t2 = theta * theta;
    float const t4 = t2 * t2;
    float const t6 = t4 * t2;
    float const t8 = t4 * t4;
    float const theta_d = theta * ( 1.f + intr.coeffs[0] * t2 + intr.coeffs[1] * t4
                                       + intr.coeffs[2] * t6 + intr.coeffs[3] * t8 );

    float const scale = theta_d / r;
    u = intr.fx * ( x * scale ) + intr.ppx;
    v = intr.fy * ( y * scale ) + intr.ppy;
    return true;
}

}  // namespace


remap_table build_remap( rs2_intrinsics const & intr,
                         float const rotation[9],
                         rectified_config const & cfg )
{
    remap_table table;
    table.width  = cfg.width_px();
    table.height = cfg.height_px;
    table.src_x.assign( size_t( table.width ) * table.height, -1.f );
    table.src_y.assign( size_t( table.width ) * table.height, -1.f );

    float const f  = cfg.focal_px();
    float const cx = cfg.cx();
    float const cy = cfg.cy();

    for( int v = 0; v < table.height; ++v )
    {
        for( int u = 0; u < table.width; ++u )
        {
            // Ray through this pixel in the rectified pinhole frame.
            float const rx = ( u - cx ) / f;
            float const ry = ( v - cy ) / f;
            float const rz = 1.f;

            // Into the physical camera's frame. Row-major 3x3.
            float const cxr = rotation[0] * rx + rotation[1] * ry + rotation[2] * rz;
            float const cyr = rotation[3] * rx + rotation[4] * ry + rotation[5] * rz;
            float const czr = rotation[6] * rx + rotation[7] * ry + rotation[8] * rz;

            float su, sv;
            if( ! project_kannala_brandt4( intr, cxr, cyr, czr, su, sv ) )
                continue;

            // Leave a one-pixel border so bilinear sampling never reads out of bounds.
            if( su < 0.f || sv < 0.f || su >= intr.width - 1.f || sv >= intr.height - 1.f )
                continue;

            size_t const idx = size_t( v ) * table.width + u;
            table.src_x[idx] = su;
            table.src_y[idx] = sv;
        }
    }

    return table;
}


void remap_y8( uint8_t const * src, int src_w, int src_h,
               remap_table const & table,
               uint8_t * dst )
{
    if( ! src || ! dst || ! table.valid() )
        return;

    for( int v = 0; v < table.height; ++v )
    {
        for( int u = 0; u < table.width; ++u )
        {
            size_t const idx = size_t( v ) * table.width + u;
            float const sx = table.src_x[idx];
            float const sy = table.src_y[idx];

            if( sx < 0.f )
            {
                dst[idx] = 0;
                continue;
            }

            int const x0 = int( sx );
            int const y0 = int( sy );
            float const ax = sx - x0;
            float const ay = sy - y0;

            uint8_t const * p = src + size_t( y0 ) * src_w + x0;
            float const top    = p[0] * ( 1.f - ax ) + p[1] * ax;
            float const bottom = p[src_w] * ( 1.f - ax ) + p[src_w + 1] * ax;

            dst[idx] = uint8_t( top * ( 1.f - ay ) + bottom * ay + 0.5f );
        }
    }
}


}  // namespace t265_stereo
}  // namespace librealsense
