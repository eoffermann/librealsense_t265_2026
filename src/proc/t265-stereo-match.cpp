// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "t265-stereo-match.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
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
constexpr int CENSUS_MAX  = 24;   // bits compared, hence the worst possible Hamming distance

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
    v = v - ( ( v >> 1 ) & 0x55555555u );
    v = ( v & 0x33333333u ) + ( ( v >> 2 ) & 0x33333333u );
    return int( ( ( ( v + ( v >> 4 ) ) & 0x0F0F0F0Fu ) * 0x01010101u ) >> 24 );
}

// Local variance, used by the block matcher to reject flat regions that match anywhere.
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


// =======================================================================================
// Block matching: every pixel decided independently from its own window.
// Kept because it is substantially faster than SGM and adequate on well-textured scenes.

void match_block( uint8_t const * left, uint8_t const * right, int W, int H,
                  matcher_config const & cfg, float * disp_out )
{
    size_t const N = size_t( W ) * H;
    std::fill( disp_out, disp_out + N, -1.f );

    std::vector< uint32_t > cl, cr;
    census_transform( left,  W, H, cl );
    census_transform( right, W, H, cr );

    std::vector< float > var;
    variance_map( left, W, H, cfg.window, var );

    // Right-image best match, accumulated during the left search: the cost for left pixel x
    // at disparity d is the same window pair as right pixel (x-d) at that disparity, so the
    // consistency check needs no second search.
    std::vector< int > rdisp_d( N, -1 );
    std::vector< int > rbest( N, std::numeric_limits< int >::max() );

    int const half = cfg.window / 2;
    int const D = cfg.max_disparity;

    std::vector< int > row_cost( static_cast< size_t >( W ) );
    std::vector< int > best( W ), second( W ), best_d( W );
    std::vector< int > best_prev( W ), best_next( W );

    for( int y = half; y < H - half; ++y )
    {
        std::fill( best.begin(),      best.end(),      std::numeric_limits< int >::max() );
        std::fill( second.begin(),    second.end(),    std::numeric_limits< int >::max() );
        std::fill( best_d.begin(),    best_d.end(),    -1 );
        std::fill( best_prev.begin(), best_prev.end(), 0 );
        std::fill( best_next.begin(), best_next.end(), 0 );

        std::vector< int > prev_row_cost( static_cast< size_t >( W ), 0 );

        for( int d = 0; d < D; ++d )
        {
            std::fill( row_cost.begin(), row_cost.end(), std::numeric_limits< int >::max() / 4 );

            int running = 0;
            bool primed = false;
            for( int x = half; x < W - half; ++x )
            {
                if( x - half - d < 0 ) { primed = false; continue; }

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
                    best_next[x] = 0;
                }
                else
                {
                    if( c < second[x] ) second[x] = c;
                    if( best_d[x] == d - 1 ) best_next[x] = c;
                }
            }

            prev_row_cost.swap( row_cost );
        }

        for( int x = half; x < W - half; ++x )
        {
            int const d = best_d[x];
            if( d <= 0 || d >= D - 1 ) continue;
            if( var[size_t( y ) * W + x] < float( cfg.min_variance ) ) continue;
            if( best[x] >= int( cfg.uniqueness * second[x] ) ) continue;

            float const c0 = float( best_prev[x] );
            float const c1 = float( best[x] );
            float const c2 = float( best_next[x] );
            float const denom = c0 - 2.f * c1 + c2;
            float sub = 0.f;
            if( std::fabs( denom ) > 1e-6f ) sub = 0.5f * ( c0 - c2 ) / denom;
            if( sub < -1.f || sub > 1.f ) sub = 0.f;

            disp_out[size_t( y ) * W + x] = float( d ) + sub;
        }
    }

    if( ! cfg.lr_check )
        return;

    for( int y = 0; y < H; ++y )
        for( int x = 0; x < W; ++x )
        {
            size_t const i = size_t( y ) * W + x;
            float const dl = disp_out[i];
            if( dl < 0.f ) continue;
            int const xr = int( std::lround( x - dl ) );
            if( xr < 0 || xr >= W ) { disp_out[i] = -1.f; continue; }
            int const dr = rdisp_d[size_t( y ) * W + xr];
            if( dr < 0 || std::fabs( float( dr ) - dl ) > float( cfg.lr_max_diff ) )
                disp_out[i] = -1.f;
        }
}


// =======================================================================================
// Semi-global matching
//
// Block matching decides each pixel from its own window alone, so it can only produce depth
// where there is local texture. SGM instead aggregates cost along several 1D paths crossing
// the image. Along a path the running cost for disparity d is the matching cost plus the
// cheapest way of arriving there: staying at d, stepping one disparity (penalty P1), or
// jumping anywhere at all (penalty P2). Summing over paths lets a confident match in a
// textured region pay for its neighbours across a blank one, which is exactly what block
// matching cannot do at any threshold.
//
// P2 is reduced across intensity edges, relaxing the smoothness assumption precisely where a
// genuine depth discontinuity is most likely.

void build_cost_volume( std::vector< uint32_t > const & cl,
                        std::vector< uint32_t > const & cr,
                        int W, int H, int D,
                        std::vector< uint8_t > & cost )
{
    cost.assign( size_t( W ) * H * D, uint8_t( CENSUS_MAX ) );
    for( int y = 0; y < H; ++y )
        for( int x = 0; x < W; ++x )
        {
            uint32_t const l = cl[size_t( y ) * W + x];
            size_t const base = ( size_t( y ) * W + x ) * D;
            int const dmax = std::min( D, x + 1 );
            for( int d = 0; d < dmax; ++d )
                cost[base + d] = uint8_t( hamming( l, cr[size_t( y ) * W + ( x - d )] ) );
        }
}

void aggregate_path( std::vector< uint8_t > const & cost,
                     uint8_t const * img,
                     int W, int H, int D,
                     int dx, int dy, int P1, int P2,
                     std::vector< uint16_t > & S )
{
    std::vector< uint16_t > prev( D ), curr( D );

    // Path starts are every pixel on the border this direction enters from. A diagonal
    // direction enters from two borders, so both are seeded.
    std::vector< std::pair< int, int > > starts;
    if( dx > 0 )      for( int y = 0; y < H; ++y ) starts.push_back( std::make_pair( 0, y ) );
    else if( dx < 0 ) for( int y = 0; y < H; ++y ) starts.push_back( std::make_pair( W - 1, y ) );
    if( dy > 0 )      for( int x = 0; x < W; ++x ) starts.push_back( std::make_pair( x, 0 ) );
    else if( dy < 0 ) for( int x = 0; x < W; ++x ) starts.push_back( std::make_pair( x, H - 1 ) );

    for( size_t si = 0; si < starts.size(); ++si )
    {
        int x = starts[si].first, y = starts[si].second;
        bool first = true;
        int prev_intensity = 0;
        int prev_min = 0;

        while( x >= 0 && x < W && y >= 0 && y < H )
        {
            size_t const base = ( size_t( y ) * W + x ) * D;
            int const intensity = img[size_t( y ) * W + x];

            if( first )
            {
                for( int d = 0; d < D; ++d )
                    curr[d] = cost[base + d];
                first = false;
            }
            else
            {
                int const grad = std::abs( intensity - prev_intensity );
                int const p2 = std::max( P1 + 1, P2 / ( 1 + grad / 16 ) );

                for( int d = 0; d < D; ++d )
                {
                    int best = prev[d];
                    if( d > 0 )     best = std::min( best, prev[d - 1] + P1 );
                    if( d < D - 1 ) best = std::min( best, prev[d + 1] + P1 );
                    best = std::min( best, prev_min + p2 );
                    // Subtracting the previous minimum keeps the accumulator bounded; it is a
                    // constant per pixel so it does not change which disparity wins.
                    curr[d] = uint16_t( cost[base + d] + best - prev_min );
                }
            }

            int m = curr[0];
            for( int d = 1; d < D; ++d ) m = std::min( m, int( curr[d] ) );
            prev_min = m;

            for( int d = 0; d < D; ++d )
                S[base + d] = uint16_t( S[base + d] + curr[d] );

            prev.swap( curr );
            prev_intensity = intensity;
            x += dx; y += dy;
        }
    }
}

void match_sgm( uint8_t const * left, uint8_t const * right, int W, int H,
                matcher_config const & cfg, float * disp_out )
{
    size_t const N = size_t( W ) * H;
    int const D = cfg.max_disparity;
    std::fill( disp_out, disp_out + N, -1.f );

    std::vector< uint32_t > cl, cr;
    census_transform( left,  W, H, cl );
    census_transform( right, W, H, cr );

    std::vector< uint8_t > cost;
    build_cost_volume( cl, cr, W, H, D, cost );

    std::vector< uint16_t > S( N * D, 0 );

    static int const dirs[8][2] = { {1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,-1},{1,-1},{-1,1} };
    int const npaths = ( cfg.sgm_paths >= 8 ) ? 8 : 4;
    for( int i = 0; i < npaths; ++i )
        aggregate_path( cost, left, W, H, D, dirs[i][0], dirs[i][1], cfg.p1, cfg.p2, S );

    // Winner-take-all over the aggregated volume. The right image's winner comes from the
    // same volume read along the other diagonal, so the consistency check is nearly free.
    std::vector< int > rbest_d( N, -1 );
    std::vector< int > rbest_c( N, std::numeric_limits< int >::max() );

    for( int y = 0; y < H; ++y )
    {
        for( int x = 0; x < W; ++x )
        {
            size_t const base = ( size_t( y ) * W + x ) * D;
            int best = std::numeric_limits< int >::max();
            int second = std::numeric_limits< int >::max();
            int bd = -1;
            int const dmax = std::min( D, x + 1 );

            for( int d = 0; d < dmax; ++d )
            {
                int const c = S[base + d];
                if( c < best ) { second = best; best = c; bd = d; }
                else if( c < second ) second = c;

                size_t const ri = size_t( y ) * W + ( x - d );
                if( c < rbest_c[ri] ) { rbest_c[ri] = c; rbest_d[ri] = d; }
            }

            if( bd <= 0 || bd >= dmax - 1 )
                continue;
            if( second != std::numeric_limits< int >::max()
                && best >= int( cfg.uniqueness * second ) )
                continue;

            float const c0 = float( S[base + bd - 1] );
            float const c1 = float( S[base + bd] );
            float const c2 = float( S[base + bd + 1] );
            float const denom = c0 - 2.f * c1 + c2;
            float sub = 0.f;
            if( std::fabs( denom ) > 1e-6f ) sub = 0.5f * ( c0 - c2 ) / denom;
            if( sub < -1.f || sub > 1.f ) sub = 0.f;

            disp_out[size_t( y ) * W + x] = float( bd ) + sub;
        }
    }

    if( ! cfg.lr_check )
        return;

    for( int y = 0; y < H; ++y )
        for( int x = 0; x < W; ++x )
        {
            size_t const i = size_t( y ) * W + x;
            float const dl = disp_out[i];
            if( dl < 0.f ) continue;
            int const xr = int( std::lround( x - dl ) );
            if( xr < 0 || xr >= W ) { disp_out[i] = -1.f; continue; }
            int const dr = rbest_d[size_t( y ) * W + xr];
            if( dr < 0 || std::fabs( float( dr ) - dl ) > float( cfg.lr_max_diff ) )
                disp_out[i] = -1.f;
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

    if( cfg.use_sgm )
        match_sgm( left, right, W, H, cfg, disp_out );
    else
        match_block( left, right, W, H, cfg, disp_out );
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
        if( d <= 0.f ) { depth_out[i] = 0; continue; }

        float const z = fb / d;
        if( z < min_z || z > max_z ) { depth_out[i] = 0; continue; }

        float const raw = z / depth_units;
        depth_out[i] = raw > 65535.f ? 0 : uint16_t( raw + 0.5f );
    }
}


}  // namespace t265_stereo
}  // namespace librealsense
