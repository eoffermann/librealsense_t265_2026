// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "t265-stereo-match.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace librealsense {
namespace t265_stereo {

namespace {

// 5x5 census transform: each pixel becomes a 24-bit word recording whether each neighbour is
// brighter than the centre. Comparing these by Hamming distance makes matching depend on local
// structure rather than absolute intensity, which is what lets the two independently-exposed
// fisheyes be compared at all.
constexpr int CENSUS_HALF = 2;

void census_transform( uint8_t const * src, int W, int H, std::vector< uint32_t > & out )
{
    out.assign( size_t( W ) * H, 0 );
    for( int y = CENSUS_HALF; y < H - CENSUS_HALF; ++y )
    {
        for( int x = CENSUS_HALF; x < W - CENSUS_HALF; ++x )
        {
            uint8_t const centre = src[size_t( y ) * W + x];
            uint32_t bits = 0;
            for( int dy = -CENSUS_HALF; dy <= CENSUS_HALF; ++dy )
                for( int dx = -CENSUS_HALF; dx <= CENSUS_HALF; ++dx )
                {
                    if( ! dx && ! dy )
                        continue;
                    bits <<= 1;
                    if( src[size_t( y + dy ) * W + ( x + dx )] < centre )
                        bits |= 1u;
                }
            out[size_t( y ) * W + x] = bits;
        }
    }
}

inline int hamming( uint32_t a, uint32_t b )
{
    uint32_t v = a ^ b;
    // popcount without relying on an intrinsic
    v = v - ( ( v >> 1 ) & 0x55555555u );
    v = ( v & 0x33333333u ) + ( ( v >> 2 ) & 0x33333333u );
    return int( ( ( ( v + ( v >> 4 ) ) & 0x0F0F0F0Fu ) * 0x01010101u ) >> 24 );
}

// Local variance, used to reject flat regions that would match anywhere.
void variance_map( uint8_t const * src, int W, int H, int win, std::vector< float > & out )
{
    int const half = win / 2;
    out.assign( size_t( W ) * H, 0.f );
    for( int y = half; y < H - half; ++y )
        for( int x = half; x < W - half; ++x )
        {
            double s = 0, s2 = 0;
            for( int dy = -half; dy <= half; ++dy )
                for( int dx = -half; dx <= half; ++dx )
                {
                    double v = src[size_t( y + dy ) * W + ( x + dx )];
                    s += v; s2 += v * v;
                }
            double const n = double( win ) * win;
            double const mean = s / n;
            out[size_t( y ) * W + x] = float( s2 / n - mean * mean );
        }
}

}  // namespace


void match_disparity( uint8_t const * left,
                      uint8_t const * right,
                      int W, int H,
                      matcher_config const & cfg,
                      float * disp_out )
{
    if( ! left || ! right || ! disp_out || W <= 0 || H <= 0 )
        return;

    size_t const N = size_t( W ) * H;
    std::fill( disp_out, disp_out + N, -1.f );

    std::vector< uint32_t > cl, cr;
    census_transform( left,  W, H, cl );
    census_transform( right, W, H, cr );

    std::vector< float > var;
    variance_map( left, W, H, cfg.window, var );

    // Right-image best match, filled opportunistically during the left search (see the
    // left-right consistency note below).
    std::vector< int > rdisp_d( N, -1 );
    std::vector< int > rbest( N, std::numeric_limits< int >::max() );

    int const half = cfg.window / 2;
    int const D = cfg.max_disparity;

    // Per-row aggregated cost for one disparity, reused across disparities.
    std::vector< int > row_cost( static_cast< size_t >( W ) );
    // best / second-best cost per pixel in the current row, and the winning disparity.
    std::vector< int > best( W ), second( W ), best_d( W );
    // Cost at d-1 and d+1 for the winner, kept for sub-pixel refinement.
    std::vector< int > cost_prev( W ), cost_next( W ), best_prev( W ), best_next( W );

    for( int y = half; y < H - half; ++y )
    {
        std::fill( best.begin(),   best.end(),   std::numeric_limits< int >::max() );
        std::fill( second.begin(), second.end(), std::numeric_limits< int >::max() );
        std::fill( best_d.begin(), best_d.end(), -1 );
        std::fill( best_prev.begin(), best_prev.end(), 0 );
        std::fill( best_next.begin(), best_next.end(), 0 );

        std::vector< int > prev_row_cost( static_cast< size_t >( W ), 0 );

        for( int d = 0; d < D; ++d )
        {
            // Per-pixel Hamming cost summed over the window rows, then a sliding sum in x.
            std::fill( row_cost.begin(), row_cost.end(), std::numeric_limits< int >::max() / 4 );

            int running = 0;
            bool primed = false;
            for( int x = half; x < W - half; ++x )
            {
                if( x - half - d < 0 )
                {
                    primed = false;
                    continue;
                }

                if( ! primed )
                {
                    running = 0;
                    for( int wy = -half; wy <= half; ++wy )
                        for( int wx = -half; wx <= half; ++wx )
                            running += hamming( cl[size_t( y + wy ) * W + ( x + wx )],
                                                cr[size_t( y + wy ) * W + ( x + wx - d )] );
                    primed = true;
                }
                else
                {
                    // Drop the column leaving the window, add the one entering it.
                    int const xout = x - half - 1;
                    int const xin  = x + half;
                    for( int wy = -half; wy <= half; ++wy )
                    {
                        running -= hamming( cl[size_t( y + wy ) * W + xout],
                                            cr[size_t( y + wy ) * W + ( xout - d )] );
                        running += hamming( cl[size_t( y + wy ) * W + xin],
                                            cr[size_t( y + wy ) * W + ( xin - d )] );
                    }
                }

                row_cost[x] = running;

                int const c = running;

                // Same window pair, viewed from the right image.
                {
                    int const xr = x - d;
                    size_t const ri = size_t( y ) * W + xr;
                    if( c < rbest[ri] ) { rbest[ri] = c; rdisp_d[ri] = d; }
                }
                if( c < best[x] )
                {
                    second[x]    = best[x];
                    best[x]      = c;
                    best_d[x]    = d;
                    best_prev[x] = prev_row_cost[x];
                    best_next[x] = 0;  // filled on the next iteration
                }
                else
                {
                    if( c < second[x] )
                        second[x] = c;
                    if( best_d[x] == d - 1 )
                        best_next[x] = c;
                }
            }

            prev_row_cost.swap( row_cost );
        }

        // Resolve winners for this row.
        for( int x = half; x < W - half; ++x )
        {
            int const d = best_d[x];
            if( d <= 0 || d >= D - 1 )
                continue;                                   // need neighbours for sub-pixel
            if( var[size_t( y ) * W + x] < float( cfg.min_variance ) )
                continue;                                   // textureless
            if( best[x] >= int( cfg.uniqueness * second[x] ) )
                continue;                                   // ambiguous

            // Parabola fit through (d-1, d, d+1) for sub-pixel disparity.
            float const c0 = float( best_prev[x] );
            float const c1 = float( best[x] );
            float const c2 = float( best_next[x] );
            float const denom = c0 - 2.f * c1 + c2;
            float sub = 0.f;
            if( std::fabs( denom ) > 1e-6f )
                sub = 0.5f * ( c0 - c2 ) / denom;
            if( sub < -1.f || sub > 1.f )
                sub = 0.f;

            disp_out[size_t( y ) * W + x] = float( d ) + sub;
        }
    }

    if( ! cfg.lr_check )
        return;

    // Left-right consistency, without a second search.
    //
    // The cost computed for left pixel x at disparity d IS the cost for right pixel (x-d) at
    // that same disparity -- it is the same pair of windows. So the right image's best match
    // is accumulated during the main pass above at no extra cost, and this pass only has to
    // compare the two answers. Recomputing it as an independent search (the obvious
    // implementation) cost about 800ms per frame for exactly the same result.
    for( int y = 0; y < H; ++y )
    {
        for( int x = 0; x < W; ++x )
        {
            size_t const i = size_t( y ) * W + x;
            float const dl = disp_out[i];
            if( dl < 0.f )
                continue;

            int const xr = int( std::lround( x - dl ) );
            if( xr < 0 || xr >= W )
            {
                disp_out[i] = -1.f;
                continue;
            }

            int const dr = rdisp_d[size_t( y ) * W + xr];
            if( dr < 0 || std::fabs( float( dr ) - dl ) > float( cfg.lr_max_diff ) )
                disp_out[i] = -1.f;
        }
    }
}


void disparity_to_depth_z16( float const * disparity,
                             int W, int H,
                             float focal_px, float baseline_m,
                             float depth_units,
                             float min_z, float max_z,
                             uint16_t * depth_out )
{
    if( ! disparity || ! depth_out )
        return;

    size_t const N = size_t( W ) * H;
    float const fb = focal_px * baseline_m;

    for( size_t i = 0; i < N; ++i )
    {
        float const d = disparity[i];
        if( d <= 0.f )
        {
            depth_out[i] = 0;
            continue;
        }

        float const z = fb / d;
        if( z < min_z || z > max_z )
        {
            depth_out[i] = 0;
            continue;
        }

        float const raw = z / depth_units;
        depth_out[i] = raw > 65535.f ? 0 : uint16_t( raw + 0.5f );
    }
}


}  // namespace t265_stereo
}  // namespace librealsense
