// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#ifndef NOMINMAX
    #define NOMINMAX
#endif

#include <memory>
#include <thread>
#include <inttypes.h> // PRIu64
#include <iomanip>
#include <sstream>
#include "tm-device.h"
#include "stream.h"
#include "media/playback/playback_device.h"
#include "media/playback/playback-device-info.h"
#include "media/ros/ros_reader.h"
#include "usb/usb-enumerator.h"
#include "tm-info.h"

// Frame types used to live in archive.h / types.h and arrived transitively. They now each
// have their own header under src/core/.
#include "core/frame-callback.h"
#include "core/frame-holder.h"
#include "core/motion-frame.h"
#include "core/pose-frame.h"
#include "core/video-frame.h"
#include "core/time-service.h"
#include "image.h"  // get_image_bpp

#include <rsutils/string/from.h>

namespace {

    // librealsense::copy_array / copy_2darray used to live in src/types.h and were removed
    // upstream. Only T265 used them, so equivalents live here rather than being pushed back
    // into shared headers. The originals took a force-narrowing flag guarded by static_assert;
    // every call site here narrows deliberately (double -> float), so the casts are explicit
    // and the flag is dropped.
    template< typename T, size_t N, typename S, size_t M >
    void copy_array( T ( &dst )[N], const S ( &src )[M] )
    {
        static_assert( N == M && N > 0, "copy_array requires equal, non-zero sizes" );
        for( size_t i = 0; i < N; ++i )
            dst[i] = static_cast< T >( src[i] );
    }

    template< typename T, size_t N1, size_t N2, typename S, size_t M1, size_t M2 >
    void copy_2darray( T ( &dst )[N1][N2], const S ( &src )[M1][M2] )
    {
        static_assert( N1 == M1 && N2 == M2 && N1 > 0 && N2 > 0,
                       "copy_2darray requires equal, non-zero sizes" );
        for( size_t i = 0; i < N1; ++i )
            for( size_t j = 0; j < N2; ++j )
                dst[i][j] = static_cast< T >( src[i][j] );
    }

}  // namespace


// uncomment to get debug messages at info severity
//#undef LOG_DEBUG
//#define LOG_DEBUG LOG_INFO

using namespace std::chrono;

#ifdef __APPLE__

#include <libkern/OSByteOrder.h>
#define htobe64(x) OSSwapHostToBigInt64(x)

#endif

static uint64_t bytesSwap(uint64_t val)
{
#if defined(__linux__) || defined(__APPLE__)
    return htobe64(val);
#else
    return _byteswap_uint64(val);
#endif
}

#include "t265-messages.h"
using namespace t265;

#include "message-print.h"

#define SENSOR_TYPE_POS (0)
#define SENSOR_TYPE_MASK (0x001F << SENSOR_TYPE_POS) /**< Bits 0-4 */
#define SENSOR_INDEX_POS (5)
#define SENSOR_INDEX_MASK (0x0007 << SENSOR_INDEX_POS) /**< Bits 5-7 */
#define SET_SENSOR_ID(_type, _index) ((((_type) << SENSOR_TYPE_POS) & SENSOR_TYPE_MASK) | (((_index) << SENSOR_INDEX_POS) & SENSOR_INDEX_MASK))
#define GET_SENSOR_INDEX(_sensorID) ((_sensorID & SENSOR_INDEX_MASK) >> SENSOR_INDEX_POS)
#define GET_SENSOR_TYPE(_sensorID) ((_sensorID & SENSOR_TYPE_MASK) >> SENSOR_TYPE_POS)


// on the MCCI (Myriad) device, endpoint numbering starts at 0
#define ENDPOINT_DEV_IN 0
#define ENDPOINT_DEV_OUT 0
#define ENDPOINT_DEV_MSGS_IN 1
#define ENDPOINT_DEV_MSGS_OUT 1
#define ENDPOINT_DEV_INT_IN 2
#define ENDPOINT_DEV_INT_OUT 2


// on host, endpoint numbering starts at 1 and has a 0x80 when the
// direction is into the host
#define ENDPOINT_HOST_IN            (ENDPOINT_DEV_OUT + 1 + 0x80)
#define ENDPOINT_HOST_OUT           (ENDPOINT_DEV_IN  + 1)
#define ENDPOINT_HOST_MSGS_IN       (ENDPOINT_DEV_MSGS_OUT + 1 + 0x80)
#define ENDPOINT_HOST_MSGS_OUT      (ENDPOINT_DEV_MSGS_IN  + 1)
#define ENDPOINT_HOST_INT_IN        (ENDPOINT_DEV_INT_OUT + 1 + 0x80)
#define ENDPOINT_HOST_INT_OUT       (ENDPOINT_DEV_INT_IN + 1)

enum log_level {
        NO_LOG        = 0x0000,
        LOG_ERR       = 0x0001,
        INFO          = 0x0002,
        WARNING       = 0x0003,
        DEBUG         = 0x0004,
        VERBOSE       = 0x0005,
        TRACE         = 0x0006,
        MAX_LOG_LEVEL
};

#define FREE_CONT 0x8001

static const int MAX_TRANSFER_SIZE = 848 * 800 + sizeof(bulk_message_video_stream); // max size on ENDPOINT_HOST_IN
static const int USB_TIMEOUT = 10000/*ms*/;
static const int BUFFER_SIZE = 1024; // Max size for control transfers

const std::map<PixelFormat, rs2_format> tm2_formats_map =
{
    { PixelFormat::ANY,           RS2_FORMAT_ANY },
    { PixelFormat::Z16,           RS2_FORMAT_Z16 },
    { PixelFormat::DISPARITY16,   RS2_FORMAT_DISPARITY16 },
    { PixelFormat::XYZ32F,        RS2_FORMAT_XYZ32F },
    { PixelFormat::YUYV,          RS2_FORMAT_YUYV },
    { PixelFormat::RGB8,          RS2_FORMAT_RGB8 },
    { PixelFormat::BGR8,          RS2_FORMAT_BGR8 },
    { PixelFormat::RGBA8,         RS2_FORMAT_RGBA8 },
    { PixelFormat::BGRA8,         RS2_FORMAT_BGRA8 },
    { PixelFormat::Y8,            RS2_FORMAT_Y8 },
    { PixelFormat::Y16,           RS2_FORMAT_Y16 },
    { PixelFormat::RAW8,          RS2_FORMAT_RAW8 },
    { PixelFormat::RAW10,         RS2_FORMAT_RAW10 },
    { PixelFormat::RAW16,         RS2_FORMAT_RAW16 }
};

inline rs2_format rs2_format_from_tm2(const uint8_t format)
{
    return tm2_formats_map.at(static_cast<const PixelFormat>(format));
}

inline int tm2_sensor_type(rs2_stream rtype)
{
    if(rtype == RS2_STREAM_FISHEYE)
        return SensorType::Fisheye;
    else if(rtype == RS2_STREAM_ACCEL)
        return SensorType::Accelerometer;
    else if(rtype == RS2_STREAM_GYRO)
        return SensorType::Gyro;
    else if(rtype == RS2_STREAM_POSE)
        return SensorType::Pose;
    else
        throw librealsense::invalid_value_exception("Invalid stream type");
}

inline int tm2_sensor_id(rs2_stream type, int stream_index)
{
    int stream_type = tm2_sensor_type(type);
    if (stream_type == SensorType::Fisheye)
        stream_index--;

    return stream_index;
}

namespace librealsense
{
    struct video_frame_metadata {
        int64_t arrival_ts;
        uint32_t exposure_time;
    };

    struct motion_frame_metadata {
        int64_t arrival_ts;
        float temperature;
    };

    struct pose_frame_metadata {
        int64_t arrival_ts;
    };

    enum temperature_type { TEMPERATURE_TYPE_ASIC, TEMPERATURE_TYPE_MOTION};

    // Renamed from temperature_option: upstream added a librealsense::temperature_option
    // in src/ds/d500/d500-options.h, and both landing in the same namespace is a link
    // error (LNK2005). This one is private to T265, so it takes the qualified name.
    class tm2_temperature_option : public readonly_option
    {
    public:
        float query() const override { return _ep.get_temperature(_type).fTemperature; }

        option_range get_range() const override { return _range; }

        bool is_enabled() const override { return true; }

        explicit tm2_temperature_option(tm2_sensor& ep, temperature_type type) : _ep(ep), _type(type),
            _range(option_range{ 0, _ep.get_temperature(_type).fThreshold, 0, 0 }) { }

    private:
        tm2_sensor&         _ep;
        temperature_type    _type;
        option_range        _range;
    };

    class exposure_option : public option_base
    {
    public:
        float query() const override { return _ep.get_exposure(); }

        void set(float value) override { return _ep.set_exposure(value); }

        const char* get_description() const override { return "Exposure"; }

        bool is_enabled() const override { return true; }

        explicit exposure_option(tm2_sensor& ep) : _ep(ep),
            option_base(option_range{ 200, 16000, 0, 200 }) { }

    private:
        tm2_sensor&         _ep;
    };

    class exposure_mode_option : public option_base
    {
    public:
        float query() const override { return !_ep.is_manual_exposure(); }

        void set(float value) override { return _ep.set_manual_exposure(1.f - value); }

        const char* get_description() const override { return "Auto-Exposure"; }

        bool is_enabled() const override { return true; }

        explicit exposure_mode_option(tm2_sensor& ep) : _ep(ep),
            option_base(option_range{ 0, 1, 1, 1 }) { }

    private:
        tm2_sensor&         _ep;
    };

    class gain_option : public option_base
    {
    public:
        float query() const override
        {
            return _ep.get_gain();
        }

        void set(float value) override
        {
            return _ep.set_gain(value);
        }

        const char* get_description() const override { return "Gain"; }

        bool is_enabled() const override { return true; }

        explicit gain_option(tm2_sensor& ep) : _ep(ep),
            option_base(option_range{ 1, 10, 0, 1 }) { }

    private:
        tm2_sensor&         _ep;
    };

    template <SIXDOF_MODE flag, SIXDOF_MODE depends_on, bool invert = false>
    class tracking_mode_option : public option_base
    {
    public:
        float query() const override { return !!(s._tm_mode & flag) ^ invert ? 1 : 0; }

        void set(float value) override {
            if (s._is_streaming)
                throw io_exception("Option is read-only while streaming");
            s._tm_mode = (!!value ^ invert) ? (s._tm_mode | flag) : (s._tm_mode & ~flag);
        }

        const char* get_description() const override { return description; }

        bool is_enabled() const override { return !depends_on || (s._tm_mode & depends_on) ? true : false; }

        bool is_read_only() const override { return s._is_streaming; }

        explicit tracking_mode_option(tm2_sensor& sensor, const char *description_) :
            s(sensor), description(description_), option_base(option_range{ 0, 1, 1, !!(sensor._tm_mode & flag) ^ invert ? 1.f : 0.f }) { }

    private:
        tm2_sensor &s;
        const char *description;
    };

    class asic_temperature_option : public tm2_temperature_option
    {
    public:
        const char* get_description() const override
        {
            return "Current T265 Asic Temperature (degree celsius)";
        }
        explicit asic_temperature_option(tm2_sensor& ep) :tm2_temperature_option(ep, temperature_type::TEMPERATURE_TYPE_ASIC) { }
    };

    class motion_temperature_option : public tm2_temperature_option
    {
    public:

        const char* get_description() const override
        {
            return "Current T265 IMU Temperature (degree celsius)";
        }
        explicit motion_temperature_option(tm2_sensor& ep) :tm2_temperature_option(ep, temperature_type::TEMPERATURE_TYPE_MOTION) { }
    };

    class md_tm2_parser : public md_attribute_parser_base
    {
    public:
        md_tm2_parser(rs2_frame_metadata_value type) : _type(type) {}
        // md_attribute_parser_base used to declare get() and supports() separately; it now
        // has a single find() that reports presence and value together. The two old methods
        // are merged here. Note they disagreed in the original: supports() omitted pose
        // frames for TIME_OF_ARRIVAL while get() handled them. The union is used, so pose
        // frames now report an arrival time instead of silently having none.
        bool find( const frame & frm, rs2_metadata_type * p_value ) const override
        {
            if( _type == RS2_FRAME_METADATA_ACTUAL_EXPOSURE )
            {
                if( dynamic_cast< const video_frame * >( &frm ) )
                {
                    auto md = reinterpret_cast< const video_frame_metadata * >(
                        frm.additional_data.metadata_blob.data() );
                    if( p_value )
                        *p_value = (rs2_metadata_type)( md->exposure_time );
                    return true;
                }
                return false;
            }

            if( _type == RS2_FRAME_METADATA_TEMPERATURE )
            {
                if( dynamic_cast< const motion_frame * >( &frm ) )
                {
                    auto md = reinterpret_cast< const motion_frame_metadata * >(
                        frm.additional_data.metadata_blob.data() );
                    if( p_value )
                        *p_value = (rs2_metadata_type)( md->temperature );
                    return true;
                }
                return false;
            }

            if( _type == RS2_FRAME_METADATA_TIME_OF_ARRIVAL )
            {
                // additional_data.system_time is the arrival time; backend_time is what has
                // traditionally been called system_time.
                if( dynamic_cast< const video_frame * >( &frm ) || dynamic_cast< const motion_frame * >( &frm )
                    || dynamic_cast< const pose_frame * >( &frm ) )
                {
                    if( p_value )
                        *p_value = (rs2_metadata_type)( frm.additional_data.system_time );
                    return true;
                }
                return false;
            }

            if( _type == RS2_FRAME_METADATA_FRAME_TIMESTAMP )
            {
                if( dynamic_cast< const video_frame * >( &frm ) || dynamic_cast< const motion_frame * >( &frm )
                    || dynamic_cast< const pose_frame * >( &frm ) )
                {
                    if( p_value )
                        *p_value = (rs2_metadata_type)( frm.additional_data.timestamp * 1e+3 );
                    return true;
                }
                return false;
            }

            return false;
        }

    private:
        rs2_frame_metadata_value _type;
    };

    tm2_sensor::tm2_sensor(tm2_device* owner)
        : sensor_base("Tracking Module", owner), _device(owner)
    {
        LOG_DEBUG("Making a sensor " << this);
        // frame_source now keys archives by (stream, index, extension) rather than by
        // extension alone, so this limit applies PER ARCHIVE, not across the sensor. T265
        // pushes two fisheye streams plus pose and motion through this one sensor, which
        // used to share a single video archive and no longer does. Kept at the original
        // value for now; if frames are dropped under load this is the first thing to look at.
        _source.set_max_publish_list_size(256);
        _data_dispatcher = std::make_shared<dispatcher>(256); // make a queue of the same size to dispatch data messages
        _data_dispatcher->start();
        register_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE, std::make_shared<md_tm2_parser>(RS2_FRAME_METADATA_ACTUAL_EXPOSURE));
        register_metadata(RS2_FRAME_METADATA_TEMPERATURE    , std::make_shared<md_tm2_parser>(RS2_FRAME_METADATA_TEMPERATURE));
        //Replacing md parser for TIME_OF_ARRIVAL and FRAME_TIMESTAMP
        _metadata_parsers->erase(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
        _metadata_parsers->erase(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
        register_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL,std::make_shared<md_tm2_parser>(RS2_FRAME_METADATA_TIME_OF_ARRIVAL));
        register_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP, std::make_shared<md_tm2_parser>(RS2_FRAME_METADATA_FRAME_TIMESTAMP));

        // Set log level
        bulk_message_request_log_control log_request = {{ sizeof(log_request), DEV_LOG_CONTROL }};
        log_request.bVerbosity = log_level::LOG_ERR;
        log_request.bLogMode = 0x1; // rollover mode
        bulk_message_response_log_control log_response = {};
        _device->bulk_request_response(log_request, log_response);

        // start log thread
        _log_poll_thread_stop = false;
        _log_poll_thread = std::thread(&tm2_sensor::log_poll, this);

        // start time sync thread
        last_ts = { std::chrono::duration<double, std::milli>(0) };
        device_to_host_ns = 0;
        _time_sync_thread_stop = false;
        _time_sync_thread = std::thread(&tm2_sensor::time_sync, this);
    }

    tm2_sensor::~tm2_sensor()
    {
        // We have to do everything in dispose() because the tm2_device gets destroyed
        // before this destructor gets called (since it is a member
        // variable)
    }

    void tm2_sensor::dispose()
    {
        // It is important not to throw here because this gets called
        // from ~tm2_device
        try {
            _data_dispatcher->stop();
            // Use this as a proxy to know if we are still able to communicate with the device
            bool device_valid = _stream_request && _interrupt_request;

            if (device_valid && _is_streaming)
                stop();

            if (device_valid && _is_opened)
                close();

            _time_sync_thread_stop = true;
            _time_sync_thread.join();

            if (device_valid) {
                stop_stream();
                stop_interrupt();
            }

            _log_poll_thread_stop = true;
            _log_poll_thread.join();
        }
        catch (...) {
            LOG_ERROR("An error has occurred while disposing the sensor!");
        }

    }

    //sensor
    ////////
    stream_profiles tm2_sensor::init_stream_profiles()
    {
        stream_profiles results;
        std::map<uint8_t, std::shared_ptr<stream_profile_interface> > profile_map;

        bulk_message_request_get_supported_raw_streams request = {{ sizeof(request), DEV_GET_SUPPORTED_RAW_STREAMS }};
        char buffer[BUFFER_SIZE];
        auto response = (bulk_message_response_get_supported_raw_streams *)buffer;
        _device->bulk_request_response(request, *response, BUFFER_SIZE);

        // Note the pose stream is special. We need to create one for
        // librealsense below, but it is not a raw_stream as defined
        // by our current protocol.

        _supported_raw_streams.clear();
        for(int i = 0; i < response->wNumSupportedStreams; i++) {
            auto tm_stream = response->stream[i];

            auto sensor_type = GET_SENSOR_TYPE(tm_stream.bSensorID);
            auto sensor_id   = GET_SENSOR_INDEX(tm_stream.bSensorID);
            if(sensor_type == SensorType::Fisheye) {
                if (sensor_id > 1) {
                    LOG_ERROR("Fisheye with sensor id > 1: " << sensor_id);
                    continue;
                }

                rs2_stream stream = RS2_STREAM_FISHEYE;
                rs2_format format = rs2_format_from_tm2(tm_stream.bPixelFormat);
                platform::stream_profile p = { tm_stream.wWidth, tm_stream.wHeight, tm_stream.wFramesPerSecond, uint32_t(format) };
                auto profile = std::make_shared<video_stream_profile>();
                profile->set_dims(p.width, p.height);
                profile->set_stream_type(stream);
                profile->set_stream_index(sensor_id + 1);  // for nice presentation by the viewer - add 1 to stream index
                profile->set_format(format);
                profile->set_framerate(p.fps);
                profile->set_unique_id(environment::get_instance().generate_stream_id());
                stream_profile sp = { profile->get_format(), stream, profile->get_stream_index(), p.width, p.height, p.fps };
                auto intrinsics = get_intrinsics(sp);

                profile->set_intrinsics([intrinsics]() { return intrinsics; });
                profile->tag_profile(profile_tag::PROFILE_TAG_DEFAULT | profile_tag::PROFILE_TAG_SUPERSET);
                results.push_back(profile);
                profile_map[tm_stream.bSensorID] = profile;

                LOG_INFO("Added a fisheye sensor id: " << sensor_id);
            }
            else if(sensor_type == SensorType::Gyro || sensor_type == SensorType::Accelerometer) {
                std::string sensor_str = sensor_type == SensorType::Gyro ? "gyro" : "accel";
                if(sensor_id != 0) {
                    LOG_ERROR(sensor_str << " with sensor id != 0: " << sensor_id);
                    continue;
                }

                rs2_format format = RS2_FORMAT_MOTION_XYZ32F;
                rs2_stream stream = (sensor_type == SensorType::Gyro) ? RS2_STREAM_GYRO : RS2_STREAM_ACCEL;
                if(stream == RS2_STREAM_GYRO && tm_stream.wFramesPerSecond != 200) {
                    LOG_DEBUG("Skipping gyro FPS " << tm_stream.wFramesPerSecond);
                    continue;
                }
                if(stream == RS2_STREAM_ACCEL && tm_stream.wFramesPerSecond != 62) {
                    LOG_DEBUG("Skipping accel FPS " << tm_stream.wFramesPerSecond);
                    continue;
                }
                auto profile = std::make_shared<motion_stream_profile>();
                profile->set_stream_type(stream);
                profile->set_stream_index(sensor_id); // for nice presentation by the viewer - add 1 to stream index
                profile->set_format(format);
                profile->set_framerate(tm_stream.wFramesPerSecond);
                profile->set_unique_id(environment::get_instance().generate_stream_id());
                auto intrinsics = get_motion_intrinsics(*profile);
                profile->set_intrinsics([intrinsics]() { return intrinsics; });

                profile->tag_profile(profile_tag::PROFILE_TAG_DEFAULT | profile_tag::PROFILE_TAG_SUPERSET);
                results.push_back(profile);
                profile_map[tm_stream.bSensorID] = profile;

                LOG_INFO("Added a " << sensor_str << " sensor id: " << sensor_id << " at " << tm_stream.wFramesPerSecond << "Hz");
            }
            else if(sensor_type == SensorType::Velocimeter) {
                LOG_INFO("Skipped a velocimeter stream");
                // Nothing to do for velocimeter streams yet
                continue;
            }
            // This one should never show up, because the device doesn't
            // actually stream poses over the bulk endpoint
            else if(sensor_type == SensorType::Pose) {
                LOG_ERROR("Found a pose stream but should not have one here");
                continue;
            }
            else if(sensor_type == SensorType::Mask) {
                //TODO: implement Mask
                continue;
            }
            else {
                LOG_ERROR("Invalid stream type " << sensor_type << " with sensor id: " << sensor_id);
                throw io_exception("Invalid stream type");
            }

            _supported_raw_streams.push_back(tm_stream);
        }
        LOG_DEBUG("Total streams " << _supported_raw_streams.size());

        // Add a "special" pose stream profile, because we handle this
        // one differently and it is implicitly always available
        rs2_format format = RS2_FORMAT_6DOF;
        const uint32_t pose_fps = 200;
        auto profile = std::make_shared<pose_stream_profile>();
        profile->set_stream_type(RS2_STREAM_POSE);
        profile->set_stream_index(0);
        profile->set_format(format);
        profile->set_framerate(pose_fps);
        profile->set_unique_id(environment::get_instance().generate_stream_id());

        profile->tag_profile(profile_tag::PROFILE_TAG_DEFAULT | profile_tag::PROFILE_TAG_SUPERSET);
        // The RS2_STREAM_POSE profile is the reference for extrinsics
        auto reference_profile = profile;
        results.push_back(profile);
        profile_map[SET_SENSOR_ID(SensorType::Pose, 0)] = profile;

        // Add a synthesised depth stream, computed on the host from the fisheye pair.
        // Like pose this is not a raw stream the device can be asked for. It is only
        // offered when the device actually has two fisheye cameras to work with.
        {
            int fisheye_count = 0;
            for( auto const & raw : _supported_raw_streams )
                if( GET_SENSOR_TYPE( raw.bSensorID ) == SensorType::Fisheye )
                    ++fisheye_count;

            if( fisheye_count >= 2 )
            {
                auto depth = std::make_shared< video_stream_profile >();
                depth->set_stream_type( RS2_STREAM_DEPTH );
                depth->set_stream_index( 0 );
                depth->set_format( RS2_FORMAT_Z16 );
                depth->set_framerate( 30 );
                depth->set_dims( _stereo_cfg.width_px() - _stereo_cfg.max_disparity,
                                 _stereo_cfg.height_px );
                depth->set_unique_id( environment::get_instance().generate_stream_id() );

                // Rectified output is an ideal pinhole; the principal point shifts because
                // the disparity search margin is cropped before publishing.
                auto intr = t265_stereo::rectified_intrinsics( _stereo_cfg );
                intr.width -= _stereo_cfg.max_disparity;
                intr.ppx   -= _stereo_cfg.max_disparity;
                depth->set_intrinsics( [intr]() { return intr; } );

                depth->tag_profile( profile_tag::PROFILE_TAG_SUPERSET );
                results.push_back( depth );
                _depth_profile = depth;
            }
        }

        //add extrinsic parameters
        for (auto profile : results)
        {
            auto current_extrinsics = get_extrinsics(*profile, 0); // TODO remove 0
            environment::get_instance().get_extrinsics_graph().register_extrinsics(*profile, *reference_profile, current_extrinsics);
        }

        auto accel_it = std::find_if(results.begin(), results.end(),
            [](std::shared_ptr<stream_profile_interface> spi) { return RS2_STREAM_ACCEL == spi->get_stream_type(); });
        auto gyro_it = std::find_if(results.begin(), results.end(),
            [](std::shared_ptr<stream_profile_interface> spi) { return RS2_STREAM_GYRO == spi->get_stream_type(); });
        if ((accel_it != results.end()) && (gyro_it != results.end()))
            environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*(accel_it->get()), *(gyro_it->get()));
        return results;
    }

    void tm2_sensor::open(const stream_profiles& requests)
    {
        LOG_DEBUG("T265 open");
        std::lock_guard<std::mutex> lock(_tm_op_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("open(...) failed. T265 device is streaming!");
        else if (_is_opened)
            throw wrong_api_call_sequence_exception("open(...) failed. T265 device is already opened!");

        _source.init(_metadata_parsers);
        _source.set_sensor(this->shared_from_this());

        //TODO - TM2_API currently supports a single profile is supported per stream.
        // this function uses stream index to determine stream profile, and just validates the requested profile.
        // need to fix this to search among supported profiles per stream. ( use sensor_base::resolve_requests?)
        if (_loopback)
        {
            auto& loopback_sensor = _loopback->get_sensor(0);
            //TODO: Future work: filter out raw streams according to requested output.
            loopback_sensor.open(loopback_sensor.get_stream_profiles());
        }

        _active_raw_streams.clear();
        for(auto p : _supported_raw_streams) {
            p.bOutputMode = 0; // disable output
            _active_raw_streams.push_back(p);
        }

        for (auto&& r : requests) {
            auto sp = to_profile(r.get());
            int stream_index = sp.index;
            rs2_stream stream_type = r->get_stream_type();
            int tm_sensor_type = tm2_sensor_type(stream_type);
            int tm_sensor_id   = tm2_sensor_id(stream_type, stream_index);
            LOG_INFO("Request for stream type " << r->get_stream_type() << " with stream index " << stream_index);

            if(stream_type == RS2_STREAM_POSE) {
                if(stream_index != 0)
                    throw invalid_value_exception("Invalid profile configuration - pose stream only supports index 0");
                LOG_DEBUG("Pose output enabled");
                _pose_output_enabled = true;
                continue;
            }

            if(stream_type == RS2_STREAM_DEPTH) {
                // Depth is derived from the fisheye pair, so both fisheye raw streams have
                // to be switched on whether or not the caller asked for them. They are
                // enabled below, after the requested streams have been processed, so an
                // explicit fisheye request still takes precedence for format and rate.
                LOG_DEBUG("Host-side stereo depth enabled");
                _depth_output_enabled = true;
                continue;
            }

            bool found = false;
            for(auto & tm_profile : _active_raw_streams) {
                if(GET_SENSOR_TYPE(tm_profile.bSensorID) == tm_sensor_type &&
                   GET_SENSOR_INDEX(tm_profile.bSensorID) == tm_sensor_id &&
                   tm_profile.wFramesPerSecond == sp.fps) {

                    if(tm_sensor_type != SensorType::Fisheye ||
                        (tm_profile.wWidth == sp.width && tm_profile.wHeight == sp.height &&
                         rs2_format_from_tm2(tm_profile.bPixelFormat) == sp.format)) {
                        if(_device->usb_info.conn_spec < platform::usb3_type)
                            LOG_ERROR("Streaming T265 video over USB " << platform::usb_spec_names.at(_device->usb_info.conn_spec) << " is unreliable, please use USB 3 or only stream poses");

                        found = true;
                        tm_profile.bOutputMode = 1;
                        break;
                    }
                }
            }
            if(!found)
                throw invalid_value_exception("Invalid profile configuration - no matching stream");
        }

        if( _depth_output_enabled )
        {
            // Enable both fisheye streams at their native configuration if the caller did
            // not request them itself.
            for( auto & tm_profile : _active_raw_streams )
                if( GET_SENSOR_TYPE( tm_profile.bSensorID ) == SensorType::Fisheye )
                    tm_profile.bOutputMode = 1;
        }

        int fisheye_streams = 0;
        for(auto tm_profile : _active_raw_streams)
            if(GET_SENSOR_TYPE(tm_profile.bSensorID) == SensorType::Fisheye && tm_profile.bOutputMode)
                fisheye_streams++;

        if(fisheye_streams == 1)
            throw invalid_value_exception("Invalid profile configuration - setting a single FE stream is not supported");

        int num_active = 0;
        for(auto p : _active_raw_streams)
            if(p.bOutputMode)
                num_active++;
        if(_pose_output_enabled) num_active++;
        LOG_INFO("Activated " << num_active << "/" << _active_raw_streams.size() + 1 << " streams");

        // Construct message to the FW to activate the streams we want
        char buffer[BUFFER_SIZE];
        auto request = (bulk_message_request_raw_streams_control *)buffer;
        request->header.wMessageID = _loopback ? DEV_RAW_STREAMS_PLAYBACK_CONTROL : DEV_RAW_STREAMS_CONTROL;
        request->wNumEnabledStreams = uint32_t(_active_raw_streams.size());
        memcpy(request->stream, _active_raw_streams.data(), request->wNumEnabledStreams*sizeof(supported_raw_stream_libtm_message));
        request->header.dwLength = request->wNumEnabledStreams * sizeof(supported_raw_stream_libtm_message) + sizeof(request->header) + sizeof(request->wNumEnabledStreams);
        bulk_message_response_raw_streams_control response;
        for(int i = 0; i < 5; i++) {
            _device->bulk_request_response(*request, response, sizeof(response), false);
            if(response.header.wStatus == DEVICE_BUSY) {
                LOG_WARNING("Device is busy, trying again");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            else if(response.header.wStatus == INVALID_REQUEST_LEN)
                throw io_exception("open(...) failed. Invalid stream request packet");
            else if(response.header.wStatus == INVALID_PARAMETER)
                throw io_exception("open(...) failed. Invalid stream specification");
            else if(response.header.wStatus != SUCCESS)
                throw io_exception(rsutils::string::from() << "open(...) unknown error " << status_name(response.header));
            else
                break;
        }
        if(response.header.wStatus == DEVICE_BUSY)
            throw wrong_api_call_sequence_exception("open(...) failed to configure streams. T265 is running!");

        bulk_message_request_6dof_control control_request = {{ sizeof(control_request), SLAM_6DOF_CONTROL }};
        control_request.bEnable = _pose_output_enabled;
        control_request.bMode = _tm_mode;
        bulk_message_response_6dof_control control_response = {};
        _device->bulk_request_response(control_request, control_response, sizeof(control_response), false);
        if(control_response.header.wStatus == DEVICE_BUSY)
            throw wrong_api_call_sequence_exception("open(...) failed. T265 is running!");
        else if(control_response.header.wStatus != SUCCESS)
            throw io_exception(rsutils::string::from() << "open(...) unknown error opening " << status_name(response.header));

        _is_opened = true;
        set_active_streams(requests);
    }

    void tm2_sensor::close()
    {
        std::lock_guard<std::mutex> lock(_tm_op_lock);
        LOG_DEBUG("T265 close");
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("close() failed. T265 device is streaming!");
        else if (!_is_opened)
            throw wrong_api_call_sequence_exception("close() failed. T265 device was not opened!");

        if (_loopback) {
            auto& loopback_sensor = _loopback->get_sensor(0);
            loopback_sensor.close();
        }

        //reset active profiles
        _active_raw_streams.clear();
        _pose_output_enabled = false;
        _depth_output_enabled = false;

        _is_opened = false;
        set_active_streams({});
    }

    void tm2_sensor::pass_frames_to_fw(frame_holder fref)
    {
        //TODO
        /*
        auto to_nanos = [](rs2_time_t t) { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<rs2_time_t, std::milli>(t)).count(); };
        auto frame_ptr = fref.frame;
        auto get_md_or_default = [frame_ptr](rs2_frame_metadata_value md) {
            rs2_metadata_type v = 0;
            if (frame_ptr->supports_frame_metadata(md))
                v = frame_ptr->get_frame_metadata(md);
            return v;
        };


        int stream_index = fref.frame->get_stream()->get_stream_index() - 1;
        auto lrs_stream = fref.frame->get_stream()->get_stream_type();
        SensorType tm_stream;
        if (!try_convert(lrs_stream, tm_stream))
        {
            LOG_ERROR("Failed to convert lrs stream " << lrs_stream << " to tm stream");
            return;
        }

        //Pass frames to firmware
        if (auto vframe = As<video_frame>(fref.frame))
        {
            TrackingData::VideoFrame f = {};
            f.sensorIndex = stream_index;
            //TODO: Librealsense frame numbers are 64 bit. This is a potentional interface gap
            f.frameId = vframe->additional_data.frame_number;
            f.profile.set(vframe->get_width(), vframe->get_height(), vframe->get_stride(), convertToTm2PixelFormat(vframe->get_stream()->get_format()));
            f.exposuretime = get_md_or_default(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            f.frameLength = vframe->get_height()*vframe->get_stride()* (vframe->get_bpp() / 8);
            f.data = vframe->data.data();
            f.timestamp = to_nanos(vframe->additional_data.timestamp);
            f.systemTimestamp = to_nanos(vframe->additional_data.backend_timestamp);
            f.arrivalTimeStamp = to_nanos(vframe->additional_data.system_time);
            auto sts = _tm_dev->SendFrame(f);
            if (sts != Status::SUCCESS)
            {
                LOG_ERROR("Failed to send video frame to TM");
            }
        }
        else if (auto mframe = As<motion_frame>(fref.frame))
        {
            auto st = mframe->get_stream()->get_stream_type();
            if (st == RS2_STREAM_ACCEL)
            {
                TrackingData::AccelerometerFrame f{};
                auto mdata = reinterpret_cast<float*>(mframe->data.data());
                f.acceleration.set(mdata[0], mdata[1], mdata[2]);
                f.frameId = mframe->additional_data.frame_number;
                f.sensorIndex = stream_index;
                f.temperature = get_md_or_default(RS2_FRAME_METADATA_TEMPERATURE);
                f.timestamp = to_nanos(mframe->additional_data.timestamp);
                f.systemTimestamp = to_nanos(mframe->additional_data.backend_timestamp);
                f.arrivalTimeStamp = to_nanos(mframe->additional_data.system_time);
                auto sts = _tm_dev->SendFrame(f);
                if (sts != Status::SUCCESS)
                {
                    LOG_ERROR("Failed to send video frame to TM");
                }
            }
            else if(st == RS2_STREAM_GYRO)
            {
                TrackingData::GyroFrame f{};
                auto mdata = reinterpret_cast<float*>(mframe->data.data());
                f.angularVelocity.set(mdata[0], mdata[1], mdata[2]);
                f.frameId = mframe->additional_data.frame_number;
                f.sensorIndex = stream_index;
                f.temperature = get_md_or_default(RS2_FRAME_METADATA_TEMPERATURE);
                f.timestamp = to_nanos(mframe->additional_data.timestamp);
                f.systemTimestamp = to_nanos(mframe->additional_data.backend_timestamp);
                f.arrivalTimeStamp = to_nanos(mframe->additional_data.system_time);
                auto sts = _tm_dev->SendFrame(f);
                if (sts != Status::SUCCESS)
                {
                    LOG_ERROR("Failed to send video frame to TM");
                }
            }
            else if (auto mframe = As<pose_frame>(fref.frame))
            {
                //Ignore
            }
            else
            {
                LOG_ERROR("Unhandled motion frame type");
            }
        }
        else
        {
            LOG_ERROR("Unhandled frame type");
        }
        */
    }

    void tm2_sensor::start(rs2_frame_callback_sptr callback)
    {
        std::lock_guard<std::mutex> lock(_tm_op_lock);
        LOG_DEBUG("Starting T265");
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. T265 device is already streaming!");
        else if (!_is_opened)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. T265 device was not opened!");

        start_interrupt();
        start_stream();

        _source.set_callback(callback);
        raise_on_before_streaming_changes(true);

        if( _depth_output_enabled )
            start_stereo_worker();

        bulk_message_request_start request = {{ sizeof(request), DEV_START }};
        bulk_message_response_start response = {};
        _device->bulk_request_response(request, response, sizeof(response), false);
        if(response.header.wStatus == DEVICE_BUSY)
            throw wrong_api_call_sequence_exception("open(...) failed. T265 is already started!");
        else if(response.header.wStatus != SUCCESS)
            throw io_exception(rsutils::string::from() << "open(...) unknown error starting " << status_name(response.header));

        LOG_DEBUG("T265 started");

        if (_loopback) {
            auto& loopback_sensor = _loopback->get_sensor(0);
            // internal_frame_callback<> was removed; make_frame_callback() is the
            // replacement. It takes a std::function<void(frame_interface*)>, and a lambda
            // taking frame_holder still binds because frame_holder has a non-explicit
            // (non-acquiring) constructor from frame_interface*.
            auto file_frames_callback = make_frame_callback(
                [this]( frame_holder fref ) { pass_frames_to_fw( std::move( fref ) ); } );
            loopback_sensor.start( file_frames_callback );
        }

        _is_streaming = true;
    }

    void tm2_sensor::stop()
    {
        // Outside the op lock: the worker takes _stereo_mutex and must be able to finish.
        stop_stereo_worker();

        std::lock_guard<std::mutex> lock(_tm_op_lock);
        LOG_DEBUG("Stopping T265");
        if (!_is_streaming)
            throw wrong_api_call_sequence_exception("stop_streaming() failed. T265 device is not streaming!");

        if (_loopback) {
            auto& loopback_sensor = _loopback->get_sensor(0);
            loopback_sensor.stop();
        }

        // Send the stop message
        bulk_message_request_stop request = {{ sizeof(request), DEV_STOP }};
        bulk_message_response_stop response = {};
        _device->bulk_request_response(request, response, sizeof(response), false);
        if(response.header.wStatus == TIMEOUT)
            LOG_WARNING("Got a timeout while trying to stop");
        else if(response.header.wStatus != SUCCESS)
            throw io_exception(rsutils::string::from() << "Unknown error stopping " << status_name(response.header));

        LOG_DEBUG("T265 stopped");

        stop_stream();
        stop_interrupt();

        raise_on_before_streaming_changes(false);
        _is_streaming = false;
    }

    rs2_intrinsics tm2_sensor::get_intrinsics(const stream_profile& profile) const
    {
        rs2_stream rtype = profile.stream;
        int rs2_index = profile.index;
        int sensor_type = tm2_sensor_type(rtype);
        int sensor_id = tm2_sensor_id(rtype, rs2_index);

        bulk_message_request_get_camera_intrinsics request = {{ sizeof(request), DEV_GET_CAMERA_INTRINSICS }};
        request.bCameraID = SET_SENSOR_ID(sensor_type, sensor_id);
        bulk_message_response_get_camera_intrinsics response = {};
        _device->bulk_request_response(request, response);

        rs2_intrinsics result;
        result.width = response.intrinsics.dwWidth;
        result.height = response.intrinsics.dwHeight;
        result.ppx = response.intrinsics.flPpx;
        result.ppy = response.intrinsics.flPpy;
        result.fx = response.intrinsics.flFx;
        result.fy = response.intrinsics.flFy;
        if(response.intrinsics.dwDistortionModel == 1) result.model = RS2_DISTORTION_FTHETA;
        else if(response.intrinsics.dwDistortionModel == 3) result.model = RS2_DISTORTION_NONE;
        else if(response.intrinsics.dwDistortionModel == 4) result.model = RS2_DISTORTION_KANNALA_BRANDT4;
        else
            throw invalid_value_exception("Invalid distortion model");
        copy_array(result.coeffs, response.intrinsics.flCoeffs);

        return result;
    }

    rs2_motion_device_intrinsic tm2_sensor::get_motion_intrinsics(const motion_stream_profile_interface& profile) const
    {
        rs2_stream rtype = profile.get_stream_type();
        int rs2_index = profile.get_stream_index();
        int sensor_type = tm2_sensor_type(rtype);
        int sensor_id = tm2_sensor_id(rtype, rs2_index);

        bulk_message_request_get_motion_intrinsics request = {{ sizeof(request), DEV_GET_MOTION_INTRINSICS }};
        request.bMotionID = SET_SENSOR_ID(sensor_type, sensor_id);
        bulk_message_response_get_motion_intrinsics response = {};
        _device->bulk_request_response(request, response);

        rs2_motion_device_intrinsic result{};
        copy_2darray(result.data, response.intrinsics.flData);
        copy_array(result.noise_variances, response.intrinsics.flNoiseVariances);
        copy_array(result.bias_variances, response.intrinsics.flBiasVariances);
        return result;
    }

    rs2_extrinsics tm2_sensor::get_extrinsics(const stream_profile_interface & profile, int tmsensor_id) const
    {
        rs2_stream rtype = profile.get_stream_type();
        int rs2_index = profile.get_stream_index();
        int sensor_type = tm2_sensor_type(rtype);
        int sensor_id = tm2_sensor_id(rtype, rs2_index);

        bulk_message_request_get_extrinsics request = {{ sizeof(request), DEV_GET_EXTRINSICS }};
        request.bSensorID = SET_SENSOR_ID(sensor_type, sensor_id);
        bulk_message_response_get_extrinsics response = {};
        _device->bulk_request_response(request, response);

        if(response.extrinsics.bReferenceSensorID != SET_SENSOR_ID(SensorType::Pose, 0)) {
            LOG_ERROR("Unexpected reference sensor id " << response.extrinsics.bReferenceSensorID);
        }

        rs2_extrinsics result{};
        copy_array(result.rotation, response.extrinsics.flRotation);
        copy_array(result.translation, response.extrinsics.flTranslation);

        return result;
    }

    void tm2_sensor::set_intrinsics(const stream_profile_interface& stream_profile, const rs2_intrinsics& intr)
    {
        bulk_message_request_set_camera_intrinsics request = {{ sizeof(request), DEV_SET_CAMERA_INTRINSICS }};

        int stream_index =  stream_profile.get_stream_index() - 1;
        if(stream_index != 0 && stream_index != 1)
            throw invalid_value_exception("Invalid fisheye stream");

        request.bCameraID = SET_SENSOR_ID(SensorType::Fisheye, stream_index);
        request.intrinsics.dwWidth = intr.width;
        request.intrinsics.dwHeight = intr.height;
        request.intrinsics.flFx = intr.fx;
        request.intrinsics.flFy = intr.fy;
        request.intrinsics.flPpx = intr.ppx;
        request.intrinsics.flPpy = intr.ppy;
        if(intr.model == RS2_DISTORTION_FTHETA)               request.intrinsics.dwDistortionModel = 1;
        else if(intr.model == RS2_DISTORTION_NONE)            request.intrinsics.dwDistortionModel = 3;
        else if(intr.model == RS2_DISTORTION_KANNALA_BRANDT4) request.intrinsics.dwDistortionModel = 4;
        else
            throw invalid_value_exception("Invalid distortion model");
        copy_array(request.intrinsics.flCoeffs, intr.coeffs);

        bulk_message_response_set_camera_intrinsics response = {};
        _device->bulk_request_response(request, response);
    }

    void tm2_sensor::set_extrinsics(const stream_profile_interface& from_profile, const stream_profile_interface& to_profile, const rs2_extrinsics& extr)
    {
        switch (to_profile.get_stream_type()) {
        case RS2_STREAM_POSE:
            set_extrinsics_to_ref(from_profile.get_stream_type(), from_profile.get_stream_index(), extr);
            break;
        case RS2_STREAM_FISHEYE:
        {
            auto inv = [](const rs2_extrinsics& src) {
                rs2_extrinsics dst;
                auto dst_rotation = dst.rotation;
                for (auto i : { 0,3,6,1,4,7,2,5,8 }) { *dst_rotation++ = src.rotation[i]; }
                dst.translation[0] = - src.rotation[0] * src.translation[0] - src.rotation[3] * src.translation[1] - src.rotation[6] * src.translation[2];
                dst.translation[1] = - src.rotation[1] * src.translation[0] - src.rotation[4] * src.translation[1] - src.rotation[7] * src.translation[2];
                dst.translation[2] = - src.rotation[2] * src.translation[0] - src.rotation[5] * src.translation[1] - src.rotation[8] * src.translation[2];
                return dst;
            };

            auto mult = [](const rs2_extrinsics& a, const rs2_extrinsics& b) {
                rs2_extrinsics tf;
                tf.rotation[0] = a.rotation[0] * b.rotation[0] + a.rotation[1] * b.rotation[3] + a.rotation[2] * b.rotation[6];
                tf.rotation[1] = a.rotation[0] * b.rotation[1] + a.rotation[1] * b.rotation[4] + a.rotation[2] * b.rotation[7];
                tf.rotation[2] = a.rotation[0] * b.rotation[2] + a.rotation[1] * b.rotation[5] + a.rotation[2] * b.rotation[8];
                tf.rotation[3] = a.rotation[3] * b.rotation[0] + a.rotation[4] * b.rotation[3] + a.rotation[5] * b.rotation[6];
                tf.rotation[4] = a.rotation[3] * b.rotation[1] + a.rotation[4] * b.rotation[4] + a.rotation[5] * b.rotation[7];
                tf.rotation[5] = a.rotation[3] * b.rotation[2] + a.rotation[4] * b.rotation[5] + a.rotation[5] * b.rotation[8];
                tf.rotation[6] = a.rotation[6] * b.rotation[0] + a.rotation[7] * b.rotation[3] + a.rotation[8] * b.rotation[6];
                tf.rotation[7] = a.rotation[6] * b.rotation[1] + a.rotation[7] * b.rotation[4] + a.rotation[8] * b.rotation[7];
                tf.rotation[8] = a.rotation[6] * b.rotation[2] + a.rotation[7] * b.rotation[5] + a.rotation[8] * b.rotation[8];
                tf.translation[0] = a.rotation[0] * b.translation[0] + a.rotation[1] * b.translation[1] + a.rotation[2] * b.translation[2] + a.translation[0];
                tf.translation[1] = a.rotation[3] * b.translation[0] + a.rotation[4] * b.translation[1] + a.rotation[5] * b.translation[2] + a.translation[1];
                tf.translation[2] = a.rotation[6] * b.translation[0] + a.rotation[7] * b.translation[1] + a.rotation[8] * b.translation[2] + a.translation[2];
                return tf;
            };

            auto& H_fe1_fe2 = extr;
            auto H_fe2_fe1  = inv(H_fe1_fe2);
            auto H_fe1_pose = get_extrinsics(from_profile, 0);
            auto H_fe2_pose = mult(H_fe2_fe1, H_fe1_pose);  //H_fe2_pose = H_fe2_fe1 * H_fe1_pose
            set_extrinsics_to_ref(RS2_STREAM_FISHEYE, 2, H_fe2_pose);
            break;
        }
        default:
            throw invalid_value_exception("Invalid stream type: set_extrinsics only support fisheye stream");
        }
    }

    void tm2_sensor::set_extrinsics_to_ref(rs2_stream stream_type, int stream_index, const rs2_extrinsics& extr)
    {
        int sensor_type = tm2_sensor_type(stream_type);
        int sensor_id   = tm2_sensor_id(stream_type, stream_index);

        bulk_message_request_set_extrinsics request = {{ sizeof(request), DEV_SET_EXTRINSICS }};
        request.bSensorID = SET_SENSOR_ID(sensor_type, sensor_id);
        copy_array(request.extrinsics.flRotation, extr.rotation);
        copy_array(request.extrinsics.flTranslation, extr.translation);
        bulk_message_response_set_extrinsics response = {};

        _device->bulk_request_response(request, response);
    }

    void tm2_sensor::set_motion_device_intrinsics(const stream_profile_interface& stream_profile, const rs2_motion_device_intrinsic& intr)
    {
        int sensor_type = tm2_sensor_type(stream_profile.get_stream_type());
        int sensor_id   = tm2_sensor_id(stream_profile.get_stream_type(), stream_profile.get_stream_index());

        if (sensor_id != 0 || (sensor_type != RS2_STREAM_GYRO && sensor_type != RS2_STREAM_ACCEL))
            throw invalid_value_exception("Invalid stream index");

        if(sensor_type != SensorType::Gyro || sensor_type != SensorType::Accelerometer)
            throw invalid_value_exception("Invalid stream type");

        bulk_message_request_set_motion_intrinsics request = {{ sizeof(request), DEV_SET_MOTION_INTRINSICS }};
        request.bMotionID = SET_SENSOR_ID(sensor_type, sensor_id);
        copy_2darray(request.intrinsics.flData, intr.data);
        copy_array(request.intrinsics.flNoiseVariances, intr.noise_variances);
        copy_array(request.intrinsics.flBiasVariances, intr.bias_variances);

        bulk_message_response_set_motion_intrinsics response = {};
        _device->bulk_request_response(request, response);
    }

    void tm2_sensor::write_calibration()
    {
        bulk_message_request_write_configuration request = {{ sizeof(request), DEV_WRITE_CONFIGURATION }};
        request.wTableId = ID_OEM_CAL;
        // We send an implicitly 0 length bTable and the right table
        // id to tell us to write the calibration
        request.header.dwLength = offsetof(bulk_message_request_write_configuration, bTable);
        bulk_message_response_write_configuration response = {};
        _device->bulk_request_response(request, response);
    }

    void tm2_sensor::reset_to_factory_calibration()
    {
        bulk_message_request_reset_configuration request = {{ sizeof(request), DEV_RESET_CONFIGURATION }};
        request.wTableId = ID_OEM_CAL;
        bulk_message_response_reset_configuration response = {};
        _device->bulk_request_response(request, response);
    }

    void tm2_sensor::dispatch_threaded(frame_holder frame)
    {
        // TODO: Replace with C++14 move capture
        auto frame_holder_ptr = std::make_shared<frame_holder>();
        *frame_holder_ptr = std::move(frame);
        _data_dispatcher->invoke([this, frame_holder_ptr](dispatcher::cancellable_timer t) {
                _source.invoke_callback(std::move(*frame_holder_ptr));
            });
    }

    void tm2_sensor::receive_pose_message(const interrupt_message_get_pose & pose_message)
    {
        const pose_data & pose = pose_message.pose;
        // pose stream doesn't have a frame number, so we have to keep
        // track ourselves
        static unsigned long long frame_num = 0;

        auto ts = get_coordinated_timestamp(pose.llNanoseconds);
        pose_frame_metadata frame_md = { 0 };
        frame_md.arrival_ts = duration_cast<std::chrono::nanoseconds>(ts.arrival_ts).count();

        frame_additional_data additional_data(ts.device_ts.count(), frame_num++, ts.arrival_ts.count(), sizeof(frame_md), (uint8_t*)&frame_md, ts.global_ts.count(), 0, 0, false, 0.0);

        // Find the frame stream profile
        std::shared_ptr<stream_profile_interface> profile = nullptr;
        auto profiles = get_stream_profiles();
        for (auto&& p : profiles)
        {
            if (p->get_stream_type() == RS2_STREAM_POSE &&
                p->get_stream_index() == 0) // TODO: no reason to retrieve this every time
            {
                profile = p;
                break;
            }
        }
        if (profile == nullptr)
        {
            LOG_WARNING("Dropped frame. No valid profile");
            return;
        }

        frame_holder frame = _source.alloc_frame( { profile->get_stream_type(), profile->get_stream_index(), RS2_EXTENSION_POSE_FRAME },
                                                  sizeof( librealsense::pose_frame::pose_info ),
                                                  std::move( additional_data ),
                                                  true );
        if (frame.frame)
        {
            auto pose_frame = static_cast<librealsense::pose_frame*>(frame.frame);
            frame->set_timestamp(ts.global_ts.count());
            frame->set_timestamp_domain(RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME);
            frame->set_stream(profile);

            auto info = reinterpret_cast<librealsense::pose_frame::pose_info*>(pose_frame->data.data());
            info->translation = float3{pose.flX, pose.flY, pose.flZ};
            info->velocity = float3{pose.flVx, pose.flVy, pose.flVz};
            info->acceleration = float3{pose.flAx, pose.flAy, pose.flAz};
            info->rotation = float4{pose.flQi, pose.flQj, pose.flQk, pose.flQr};
            info->angular_velocity = float3{pose.flVAX, pose.flVAY, pose.flVAZ};
            info->angular_acceleration = float3{pose.flAAX, pose.flAAY, pose.flAAZ};
            info->tracker_confidence = pose.dwTrackerConfidence;
            info->mapper_confidence = pose.dwMapperConfidence;
        }
        else
        {
            LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
            return;
        }
        dispatch_threaded(std::move(frame));
    }

    void tm2_sensor::receive_accel_message(const interrupt_message_accelerometer_stream & message)
    {
        if (!_is_streaming)
        {
            LOG_WARNING("Frame received with streaming inactive");
            return;
        }

        float3 data = { message.metadata.flAx, message.metadata.flAy, message.metadata.flAz };
        auto sensor_id = GET_SENSOR_INDEX(message.rawStreamHeader.bSensorID);
        handle_imu_frame(message.rawStreamHeader.llNanoseconds, message.rawStreamHeader.dwFrameId, RS2_STREAM_ACCEL, sensor_id, data, message.metadata.flTemperature);
    }

    void tm2_sensor::receive_gyro_message(const interrupt_message_gyro_stream & message)
    {
        if (!_is_streaming)
        {
            LOG_WARNING("Frame received with streaming inactive");
            return;
        }

        float3 data = { message.metadata.flGx, message.metadata.flGy, message.metadata.flGz };
        auto sensor_id = GET_SENSOR_INDEX(message.rawStreamHeader.bSensorID);
        handle_imu_frame(message.rawStreamHeader.llNanoseconds, message.rawStreamHeader.dwFrameId, RS2_STREAM_GYRO, sensor_id, data, message.metadata.flTemperature);
    }

    void tm2_sensor::receive_video_message(const bulk_message_video_stream * message)
    {
        if (!_is_streaming)
        {
            LOG_WARNING("Frame received with streaming inactive");
            return;
        }
        if (message->metadata.dwFrameLength == 0)
        {
            LOG_WARNING("Dropped frame. Frame length is 0");
            return;
        }

        auto bpp = get_image_bpp(RS2_FORMAT_Y8);

        auto ts = get_coordinated_timestamp(message->rawStreamHeader.llNanoseconds);

        video_frame_metadata video_md{};
        video_md.arrival_ts = duration_cast<std::chrono::nanoseconds>(ts.arrival_ts).count();
        video_md.exposure_time = message->metadata.dwExposuretime;
        frame_additional_data additional_data(ts.device_ts.count(), message->rawStreamHeader.dwFrameId, ts.arrival_ts.count(), sizeof(video_md), (uint8_t*)&video_md, ts.global_ts.count(), 0, 0, false, 0.0);

        last_exposure = message->metadata.dwExposuretime;
        last_gain = message->metadata.fGain;

        // TODO: llArrivalNanoseconds?
        // Find the frame stream profile
        int width, height, stride;
        std::shared_ptr<stream_profile_interface> profile = nullptr;
        auto profiles = get_stream_profiles();
        for (auto&& p : profiles)
        {
            if (p->get_stream_type() == RS2_STREAM_FISHEYE &&
                p->get_stream_index() == GET_SENSOR_INDEX(message->rawStreamHeader.bSensorID) + 1)
            {
                auto video = dynamic_cast<video_stream_profile_interface*>(p.get()); //this must succeed for fisheye stream
                profile = p;
                width = stride = video->get_width();
                height = video->get_height();
                break;
            }
        }
        if (profile == nullptr)
        {
            LOG_WARNING("Dropped frame. No valid profile");
            return;
        }

        // Feed the stereo depth worker. Dimensions come from the profile rather than the
        // message, which carries only the payload length. Sensor index is 0/1 on the wire;
        // librealsense presents the same cameras as fisheye 1 and 2.
        if( _depth_output_enabled )
            submit_fisheye_for_depth( GET_SENSOR_INDEX( message->rawStreamHeader.bSensorID ),
                                      message->metadata.bFrameData,
                                      width, height,
                                      ts.global_ts.count(),
                                      message->rawStreamHeader.dwFrameId );

        //Global base time sync may happen between two frames
        //Make sure 2nd frame global timestamp is not impacted.
        auto delta_dev_ts = ts.device_ts - last_ts.device_ts;
        if (delta_dev_ts < delta_dev_ts.zero())
            delta_dev_ts = -delta_dev_ts;

        if (delta_dev_ts < std::chrono::microseconds(1000))
            ts.global_ts = last_ts.global_ts + delta_dev_ts; // keep stereo pairs times in sync

        last_ts = ts;

        //TODO - extension_type param assumes not depth
        frame_holder frame = _source.alloc_frame( { profile->get_stream_type(), profile->get_stream_index(), RS2_EXTENSION_VIDEO_FRAME },
                                                  height * stride,
                                                  std::move( additional_data ),
                                                  true );
        if (frame.frame)
        {
            auto video = (video_frame*)(frame.frame);
            video->assign(width, height, stride, bpp);
            frame->set_timestamp(ts.global_ts.count());
            frame->set_timestamp_domain(RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME);
            frame->set_stream(profile);
            frame->set_sensor(this->shared_from_this()); //TODO? uvc doesn't set it?
            video->data.assign(message->metadata.bFrameData, message->metadata.bFrameData + (height * stride));
        }
        else
        {
            LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
            return;
        }
        dispatch_threaded(std::move(frame));
    }

    tm2_sensor::coordinated_ts tm2_sensor::get_coordinated_timestamp(uint64_t device_ns)
    {
        tm2_sensor::coordinated_ts result;
        auto ts_nanos = duration<uint64_t, std::nano>(device_ns);
        result.device_ts = duration<double, std::milli>(ts_nanos);
        result.global_ts = duration<double, std::milli>(ts_nanos + duration<int64_t, std::nano>(device_to_host_ns));
        result.arrival_ts = duration<double, std::milli>(time_service::get_time());
        return result;
    }

    bool tm2_sensor::start_interrupt()
    {
        std::vector<uint8_t> buffer(BUFFER_SIZE);

        if (_interrupt_request) return false;

        _interrupt_callback = std::make_shared<platform::usb_request_callback>([&](platform::rs_usb_request request) {
            uint32_t transferred = request->get_actual_length();
            if(transferred == 0) { // something went wrong, exit
                LOG_ERROR("Interrupt transfer failed, exiting");
                _interrupt_request.reset();
                return;
            }

            interrupt_message_header* header = (interrupt_message_header*)request->get_buffer().data();
            if(header->wMessageID == DEV_GET_POSE)
                receive_pose_message(*((interrupt_message_get_pose*)header));
            else if(header->wMessageID == DEV_SAMPLE) {
                if(_is_streaming) {
                    int sensor_type = GET_SENSOR_TYPE(((interrupt_message_raw_stream_header*)header)->bSensorID);
                    if(sensor_type == SensorType::Accelerometer)
                        receive_accel_message(*((interrupt_message_accelerometer_stream*)header));
                    else if(sensor_type == SensorType::Gyro)
                        receive_gyro_message(*((interrupt_message_gyro_stream*)header));
                    else if(sensor_type == SensorType::Velocimeter)
                        LOG_ERROR("Did not expect to receive a velocimeter message");
                    else
                        LOG_ERROR("Received DEV_SAMPLE with unknown type " << sensor_type);
                }
            }
            else if(header->wMessageID == SLAM_ERROR) {
                //TODO: current librs ignores these
                auto message = (interrupt_message_slam_error *)header;
                if     (message->wStatus == SLAM_ERROR_CODE_NONE)   LOG_INFO("SLAM_ERROR none");
                else if(message->wStatus == SLAM_ERROR_CODE_VISION) LOG_WARNING("SLAM_ERROR Vision");
                else if(message->wStatus == SLAM_ERROR_CODE_SPEED)  LOG_WARNING("SLAM_ERROR Speed");
                else if(message->wStatus == SLAM_ERROR_CODE_OTHER)  LOG_WARNING("SLAM_ERROR Other");
                else                                                LOG_WARNING("SLAM_ERROR Unknown");
            }
            else if(header->wMessageID == DEV_ERROR) {
                LOG_ERROR("DEV_ERROR " << status_name(*((bulk_message_response_header*)header)));
            }
            else if(header->wMessageID == DEV_STATUS) {
                auto response = (bulk_message_response_header*)header;
                if(response->wStatus == DEVICE_STOPPED)
                    LOG_DEBUG("Got device stopped message");
                else if(response->wStatus == TEMPERATURE_WARNING)
                    LOG_WARNING("T265 temperature warning");
                else
                    LOG_WARNING("Unhandled DEV_STATUS " << status_name(*response));
            }
            else if(header->wMessageID == SLAM_SET_LOCALIZATION_DATA_STREAM) {
                receive_set_localization_data_complete(*((interrupt_message_set_localization_data_stream *)header));
            }
            else if(header->wMessageID == SLAM_RELOCALIZATION_EVENT) {
                auto event = (const interrupt_message_slam_relocalization_event *)header;
                auto ts = get_coordinated_timestamp(event->llNanoseconds);
                std::string msg = rsutils::string::from() << "T2xx: Relocalization occurred. id: " << event->wSessionId <<  ", timestamp: " << ts.global_ts.count() << " ms";
                LOG_INFO(msg);
                raise_relocalization_event(msg, ts.global_ts.count());
            }
            else
                LOG_ERROR("Unknown interrupt message " <<  message_name(*((bulk_message_response_header*)header)) << " with status " << status_name(*((bulk_message_response_header*)header)));

            _device->submit_request(request);
        });

        _interrupt_request = _device->interrupt_read_request(buffer, _interrupt_callback);
        _device->submit_request(_interrupt_request);
        return true;
    }

    void tm2_sensor::stop_interrupt()
    {
        if (_interrupt_request) {
            _interrupt_callback->cancel();
            if (!_device->cancel_request(_interrupt_request))
                LOG_WARNING("Failed to cancel the T265 interrupt request; releasing it anyway");

            // Same reasoning as stop_stream(): see the note there.
            _interrupt_request.reset();
        }
    }

    bool tm2_sensor::start_stream()
    {
        std::vector<uint8_t> buffer(MAX_TRANSFER_SIZE);

        if (_stream_request) return false;

        _stream_callback = std::make_shared<platform::usb_request_callback>([&](platform::rs_usb_request request) {
            uint32_t transferred = request->get_actual_length();
            if(!transferred) {
                LOG_ERROR("Stream transfer failed, exiting");
                _stream_request.reset();
                return;
            }

            auto header = (bulk_message_raw_stream_header *)request->get_buffer().data();
            if (transferred != header->header.dwLength) {
                LOG_ERROR("Bulk received " << transferred << " but header was " << header->header.dwLength << " bytes (max_response_size was " << MAX_TRANSFER_SIZE << ")");
            }
            LOG_DEBUG("Got " << transferred << " on bulk stream endpoint");

            if(header->header.wMessageID == DEV_STATUS) {
                auto res = (bulk_message_response_header*)header;
                if(res->wStatus == DEVICE_STOPPED)
                    LOG_DEBUG("Got device stopped message");
                else
                    LOG_WARNING("Unhandled DEV_STATUS " << status_name(*res));
            }
            else if(header->header.wMessageID == SLAM_GET_LOCALIZATION_DATA_STREAM) {
                LOG_DEBUG("GET_LOCALIZATION_DATA_STREAM status " << status_name(*((bulk_message_response_header*)header)));
                receive_localization_data_chunk((interrupt_message_get_localization_data_stream *)header);
            }
            else if(header->header.wMessageID == DEV_SAMPLE) {
                if(GET_SENSOR_TYPE(header->bSensorID) == SensorType::Fisheye) {
                    if(_is_streaming)
                        receive_video_message((bulk_message_video_stream *)header);
                }
                else
                    LOG_ERROR("Unexpected DEV_SAMPLE with " << GET_SENSOR_TYPE(header->bSensorID) << " " << GET_SENSOR_INDEX(header->bSensorID));
            }
            else {
                LOG_ERROR("Unexpected message on raw endpoint " << header->header.wMessageID);
            }

            _device->submit_request(request);
        });

        _stream_request = _device->stream_read_request(buffer, _stream_callback);
        _device->submit_request(_stream_request);
        return true;
    }

    void tm2_sensor::stop_stream()
    {
        if (_stream_request) {
            _stream_callback->cancel();
            if (!_device->cancel_request(_stream_request))
                LOG_WARNING("Failed to cancel the T265 stream request; releasing it anyway");

            // Released unconditionally, and deliberately so. This used to reset only when
            // cancel succeeded, which meant a single failed cancel left _stream_request set
            // forever -- and start_stream() early-returns when it is set, so the sensor
            // silently never streamed again for the life of the process. Dropping our handle
            // is safe: the request is shared_ptr-owned, the callback has already been
            // cancelled, and at worst one request lingers until the USB layer releases it.
            _stream_request.reset();
        }
    }

    void tm2_sensor::print_logs(const std::unique_ptr<bulk_message_response_get_and_clear_event_log> & log)
    {
        int log_size = log->header.dwLength - sizeof(bulk_message_response_header);
        int n_entries = log_size / sizeof(device_event_log);
        device_event_log * entries = (device_event_log *)log->bEventLog;

        bool start_of_entry = true;
        for (size_t i = 0; i < n_entries; i++) {
            const auto & e = entries[i];
            uint64_t timestamp;
            memcpy(&timestamp, e.timestamp, sizeof(e.timestamp));
            LOG_INFO("T265 FW message: " << timestamp << ": [0x" << e.threadID << "/" << e.moduleID << ":" << e.lineNumber << "] " << e.payload);
            start_of_entry = e.eventID == FREE_CONT ? false : true;
        }
    }

    bool tm2_sensor::log_poll_once(std::unique_ptr<bulk_message_response_get_and_clear_event_log> & log_buffer)
    {
        bulk_message_request_get_and_clear_event_log log_request = {{ sizeof(log_request), DEV_GET_AND_CLEAR_EVENT_LOG }};
        bulk_message_response_get_and_clear_event_log *log_response = log_buffer.get();
        platform::usb_status usb_response = _device->bulk_request_response(log_request, *log_response, sizeof(bulk_message_response_get_and_clear_event_log), false);
        if(usb_response != platform::RS2_USB_STATUS_SUCCESS) return false;

        if(log_response->header.wStatus == INVALID_REQUEST_LEN || log_response->header.wStatus == INTERNAL_ERROR)
            LOG_ERROR("T265 log size mismatch " << status_name(log_response->header));
        else if(log_response->header.wStatus != SUCCESS)
            LOG_ERROR("Unexpected message on log endpoint " << message_name(log_response->header) << " with status " << status_name(log_response->header));

        return true;
    }

    void tm2_sensor::log_poll()
    {
        auto log_buffer = std::unique_ptr<bulk_message_response_get_and_clear_event_log>(new bulk_message_response_get_and_clear_event_log);
        while(!_log_poll_thread_stop) {
            if(log_poll_once(log_buffer)) {
                print_logs(log_buffer);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            else {
                LOG_INFO("Got bad response, stopping log_poll");
                break;
            }
        }
    }

    void tm2_sensor::time_sync()
    {
        int tried_count = 0;
        while(!_time_sync_thread_stop) {
            bulk_message_request_get_time request = {{ sizeof(request), DEV_GET_TIME }};
            bulk_message_response_get_time response = {};

            auto start = duration<double, std::milli>(time_service::get_time());
            platform::usb_status usb_response = _device->bulk_request_response(request, response);
            if(usb_response != platform::RS2_USB_STATUS_SUCCESS) {
                LOG_INFO("Got bad response, stopping time sync");
                break;
            }
            auto finish = duration<double, std::milli>(time_service::get_time());
            auto usb_delay = (finish - start) / 2;

            //If usb response takes too long, skip update. 0.25ms is 5% of 200Hz
            if (!device_to_host_ns && usb_delay >= duration<double, std::milli>(0.25))
            {
                //In case of slower USB, not to skip after a few tries.
                if (tried_count++ < 3) continue;
            }

            if (usb_delay < duration<double, std::milli>(0.25) || !device_to_host_ns)
            {
                double device_ms = (double)response.llNanoseconds*1e-6;
                auto device = duration<double, std::milli>(device_ms);
                auto diff = duration<double, std::nano>(start + usb_delay - device);
                device_to_host_ns = diff.count();
            }
            LOG_DEBUG("T265 time synced, host_ns: " << device_to_host_ns);

            // Only trigger this approximately every 500ms, but don't
            // wait 500ms to stop if we are requested to stop
            for(int i = 0; i < 10; i++)
                if(!_time_sync_thread_stop)
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void tm2_sensor::enable_loopback(std::shared_ptr<playback_device> input)
    {
        std::lock_guard<std::mutex> lock(_tm_op_lock);
        if (_is_streaming || _is_opened)
            throw wrong_api_call_sequence_exception("T2xx: Cannot enter loopback mode while device is open or streaming");
        _loopback = input;
    }

    void tm2_sensor::disable_loopback()
    {
        std::lock_guard<std::mutex> lock(_tm_op_lock);
        _loopback.reset();
    }

    bool tm2_sensor::is_loopback_enabled() const
    {
        return _loopback != nullptr;
    }

    void tm2_sensor::handle_imu_frame(unsigned long long tm_frame_ts, unsigned long long frame_number, rs2_stream stream_type, int index, float3 imu_data, float temperature)
    {
        auto ts = get_coordinated_timestamp(tm_frame_ts);

        motion_frame_metadata motion_md{};
        motion_md.arrival_ts = duration_cast<std::chrono::nanoseconds>(ts.arrival_ts).count();
        motion_md.temperature = temperature;
        frame_additional_data additional_data(ts.device_ts.count(), frame_number, ts.arrival_ts.count(), sizeof(motion_md), (uint8_t*)&motion_md, ts.global_ts.count(), 0, 0, false, 0.0);

        // Find the frame stream profile
        std::shared_ptr<stream_profile_interface> profile = nullptr;
        auto profiles = get_stream_profiles();
        for (auto&& p : profiles)
        {
            if (p->get_stream_type() == stream_type &&
                p->get_stream_index() == index)
            {
                profile = p;
                break;
            }
        }
        if (profile == nullptr)
        {
            LOG_WARNING("Dropped frame. No valid profile");
            return;
        }

        frame_holder frame = _source.alloc_frame( { profile->get_stream_type(), profile->get_stream_index(), RS2_EXTENSION_MOTION_FRAME },
                                                  3 * sizeof( float ),
                                                  std::move( additional_data ),
                                                  true );
        if (frame.frame)
        {
            auto motion_frame = static_cast<librealsense::motion_frame*>(frame.frame);
            frame->set_timestamp(ts.global_ts.count());
            frame->set_timestamp_domain(RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME);
            frame->set_stream(profile);
            auto data = reinterpret_cast<float*>(motion_frame->data.data());
            data[0] = imu_data[0];
            data[1] = imu_data[1];
            data[2] = imu_data[2];
        }
        else
        {
            LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
            return;
        }
        dispatch_threaded(std::move(frame));
    }

    void tm2_sensor::raise_relocalization_event(const std::string& msg, double timestamp_ms)
    {
        notification event{ RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION, 0, RS2_LOG_SEVERITY_INFO, msg };
        event.timestamp = timestamp_ms;
        get_notifications_processor()->raise_notification(event);
    }

    void tm2_sensor::raise_error_notification(const std::string& msg)
    {
        notification error{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, 0, RS2_LOG_SEVERITY_ERROR, msg };
        error.timestamp = duration<double, std::milli>(time_service::get_time()).count();
        get_notifications_processor()->raise_notification(error);
    }

    void tm2_sensor::receive_set_localization_data_complete(const interrupt_message_set_localization_data_stream & message)
    {
        if(_is_streaming)
            LOG_ERROR("Received SET_LOCALIZATION_DATA_COMPLETE while streaming");

        if (_async_op_status != _async_progress)
            LOG_ERROR("Received SET_LOCALIZATION_DATA_COMPLETE without a transfer in progress");

        if (message.wStatus == MESSAGE_STATUS::SUCCESS) {
            _async_op_status = _async_success;
            _async_op.notify_one();
        }
        else {
            LOG_INFO("SET_LOCALIZATION_DATA_COMPLETE error status " << status_name(*((bulk_message_response_header *)&message)));
            _async_op_status = _async_fail; // do not notify here because we may get multiple of these messages
        }
    }

    void tm2_sensor::receive_localization_data_chunk(const interrupt_message_get_localization_data_stream * chunk)
    {
        size_t bytes = chunk->header.dwLength - offsetof(interrupt_message_get_localization_data_stream, bLocalizationData);
        LOG_DEBUG("Received chunk " << chunk->wIndex << " with status " << chunk->wStatus << " length " << bytes);

        _async_op_res_buffer.reserve(_async_op_res_buffer.size() + bytes);
        auto start = (const char*)chunk->bLocalizationData;
        _async_op_res_buffer.insert(_async_op_res_buffer.end(), start, start + bytes);
        if (chunk->wStatus == SUCCESS) {
            _async_op_status = _async_success;
            _async_op.notify_one();
        }
        else if (chunk->wStatus != MORE_DATA_AVAILABLE) {
            _async_op_status = _async_fail;
            _async_op.notify_one();
        }
    }

    std::string async_op_to_string(tm2_sensor::async_op_state val)
    {
        switch (val)
        {
            case tm2_sensor::_async_init:       return "Init";
            case tm2_sensor::_async_progress:   return "In Progress";
            case tm2_sensor::_async_success:    return "Success";
            case tm2_sensor::_async_fail:       return "Fail";
            default: return (rsutils::string::from() << " Unsupported type: " << val);
        }
    }

    tm2_sensor::async_op_state tm2_sensor::perform_async_transfer(std::function<bool()> transfer_activator,
        std::function<void()> on_success, const std::string& op_description) const
    {
        std::mutex _async_op_lock;
        std::unique_lock<std::mutex> lock(_async_op_lock);

        _async_op_status = _async_progress;
        LOG_INFO(op_description << " in progress");

        bool start_success = transfer_activator();
        if(!start_success)
            return async_op_state::_async_fail;

        if (!_async_op.wait_for(lock, std::chrono::seconds(2), [this] { return _async_op_status != _async_progress;}))
            LOG_WARNING(op_description << " aborted on timeout");
        else if (_async_op_status == _async_success)
            on_success();
        else
            LOG_ERROR(op_description << " ended with status " << async_op_to_string(_async_op_status));

        auto res = _async_op_status;
        _async_op_status = _async_init;
        LOG_DEBUG(op_description << " completed with " << async_op_to_string(res));

        return res;
    }

    bool tm2_sensor::export_relocalization_map(std::vector<uint8_t>& lmap_buf) const
    {
        std::lock_guard<std::mutex> lock(_tm_op_lock);
        auto sensor = _device->get_tm2_sensor();
        bool interrupt_started = sensor->start_interrupt();
        std::shared_ptr<void> stop_interrupt(nullptr, [&](...) {
            if (interrupt_started)
                sensor->stop_interrupt();
        });
        bool stream_started = sensor->start_stream();
        std::shared_ptr<void> stop_stream(nullptr, [&](...) {
            if (stream_started)
                sensor->stop_stream();
        });

        // Export first sends SLAM_GET_LOCALIZATION_DATA on bulk
        // endpoint and gets an acknowledgement there. That triggers
        // the firmware to send map chunks via
        // SLAM_GET_LOCALIZATION_DATA_STREAM messages on the stream
        // endpoint, eventually ending in SUCCESS
        auto res = perform_async_transfer(
            [&]() {
                _async_op_res_buffer.clear();
                bulk_message_request_get_localization_data request = {{ sizeof(request), SLAM_GET_LOCALIZATION_DATA }};
                bulk_message_response_get_localization_data response = {};
                int error = _device->bulk_request_response(request, response);
                return !error;
            },
            [&](){
                lmap_buf = this->_async_op_res_buffer;
            },
            "Export localization map");

        if (res != async_op_state::_async_success)
            LOG_ERROR("Export localization map failed");

        return res == async_op_state::_async_success;
    }

    bool tm2_sensor::import_relocalization_map(const std::vector<uint8_t>& lmap_buf) const
    {
        if(_is_streaming)
            throw wrong_api_call_sequence_exception("Unable to import relocalization map while streaming");

        std::lock_guard<std::mutex> lock(_tm_op_lock);

        auto sensor = _device->get_tm2_sensor();
        bool interrupt_started = sensor->start_interrupt();
        std::shared_ptr<void> stop_interrupt(nullptr, [&](...) {
            if (interrupt_started)
                sensor->stop_interrupt();
        });
        bool stream_started = sensor->start_stream();
        std::shared_ptr<void> stop_stream(nullptr, [&](...) {
            if (stream_started)
                sensor->stop_stream();
        });

        // Import the map by sending chunks of with id SLAM_SET_LOCALIZATION_DATA_STREAM
        auto res = perform_async_transfer(
            [this, &lmap_buf]() {
                const int MAX_BIG_DATA_MESSAGE_LENGTH = 10250; //(10240 (10k) + large message header size)
                size_t chunk_length = MAX_BIG_DATA_MESSAGE_LENGTH - offsetof(bulk_message_large_stream, bPayload);
                size_t map_size = lmap_buf.size();
                std::unique_ptr<uint8_t []> buf(new uint8_t[MAX_BIG_DATA_MESSAGE_LENGTH]);
                auto message = (bulk_message_large_stream *)buf.get();

                if (map_size == 0) return false;
                size_t left_length = map_size;
                int chunk_number = 0;
                while (left_length > 0) {
                    message->header.wMessageID = SLAM_SET_LOCALIZATION_DATA_STREAM;
                    size_t chunk_size = chunk_length;
                    message->wStatus = MESSAGE_STATUS::MORE_DATA_AVAILABLE;
                    if (left_length <= chunk_length) {
                        chunk_size = left_length;
                        message->wStatus = MESSAGE_STATUS::SUCCESS;
                    }
                    message->header.dwLength = uint32_t(chunk_size + offsetof(bulk_message_large_stream, bPayload));
                    message->wIndex = chunk_number;

                    memcpy(message->bPayload, lmap_buf.data() + (map_size - left_length), chunk_size);

                    chunk_number++;
                    left_length -= chunk_size;

                    LOG_DEBUG("Sending chunk length " << chunk_size << " of map size " << map_size);
                    _device->stream_write(&message->header);
                }
                return true;
            },
            [&]() {},
            "Import localization map");

        if (res != async_op_state::_async_success)
            LOG_ERROR("Import localization map failed");

        return res == async_op_state::_async_success;
    }

    bool tm2_sensor::set_static_node(const std::string& guid, const float3& pos, const float4& orient_quat) const
    {
        bulk_message_request_set_static_node request = {{ sizeof(request), SLAM_SET_STATIC_NODE }};
        strncpy((char *)&request.bGuid[0], guid.c_str(), MAX_GUID_LENGTH-1);
        request.data.flX = pos.x;
        request.data.flY = pos.y;
        request.data.flZ = pos.z;
        request.data.flQi = orient_quat.x;
        request.data.flQj = orient_quat.y;
        request.data.flQk = orient_quat.z;
        request.data.flQr = orient_quat.w;
        bulk_message_response_set_static_node response = {};
        _device->bulk_request_response(request, response, sizeof(response), false);
        if(response.header.wStatus == INTERNAL_ERROR)
            return false; // Failed to set static node
        else if(response.header.wStatus != SUCCESS) {
            LOG_ERROR("Error: " << status_name(response.header) << " setting static node");
            return false;
        }

        return true;
    }

    bool tm2_sensor::get_static_node(const std::string& guid, float3& pos, float4& orient_quat) const
    {
        bulk_message_request_get_static_node request = {{ sizeof(request), SLAM_GET_STATIC_NODE }};
        strncpy((char *)&request.bGuid[0], guid.c_str(), MAX_GUID_LENGTH-1);
        bulk_message_response_get_static_node response = {};

        _device->bulk_request_response(request, response, sizeof(response), false);
        if(response.header.wStatus == INTERNAL_ERROR)
            return false; // Failed to get static node
        else if(response.header.wStatus != SUCCESS) {
            LOG_ERROR("Error: " << status_name(response.header) << " getting static node");
            return false;
        }

        pos = float3{response.data.flX, response.data.flY, response.data.flZ};
        orient_quat = float4{response.data.flQi, response.data.flQj, response.data.flQk, response.data.flQr};

        return true;
    }

    bool tm2_sensor::remove_static_node(const std::string& guid) const
    {
        bulk_message_request_remove_static_node request = {{ sizeof(request), SLAM_REMOVE_STATIC_NODE }};
        strncpy((char *)&request.bGuid[0], guid.c_str(), MAX_GUID_LENGTH-1);
        bulk_message_response_remove_static_node response = {};

        _device->bulk_request_response(request, response, sizeof(response), false);
        if(response.header.wStatus == INTERNAL_ERROR)
            return false; // Failed to get static node
        else if(response.header.wStatus != SUCCESS) {
            LOG_ERROR("Error: " << status_name(response.header) << " deleting static node");
            return false;
        }
        return true;
    }

    bool tm2_sensor::load_wheel_odometery_config(const std::vector<uint8_t>& odometry_config_buf) const
    {
        std::vector<uint8_t> buf;
        buf.resize(odometry_config_buf.size() + sizeof(bulk_message_request_header));

        LOG_INFO("Sending wheel odometry with " << buf.size());

        bulk_message_request_slam_append_calibration request = {{ sizeof(request), SLAM_APPEND_CALIBRATION }};
        size_t bytes = std::min(odometry_config_buf.size(), size_t(MAX_SLAM_CALIBRATION_SIZE-1));
        strncpy((char *)request.calibration_append_string, (char *)odometry_config_buf.data(), bytes);
        request.header.dwLength = uint32_t(offsetof(bulk_message_request_slam_append_calibration, calibration_append_string) + bytes);

        //TODO: There is no way on the firmware to know if this succeeds.

        _device->stream_write(&request.header);

        return true;
    }

    bool tm2_sensor::send_wheel_odometry(uint8_t wo_sensor_id, uint32_t frame_num, const float3& translational_velocity) const
    {
        bulk_message_velocimeter_stream request = {{ sizeof(request), DEV_SAMPLE }};
        request.rawStreamHeader.bSensorID = SET_SENSOR_ID(SensorType::Velocimeter, wo_sensor_id);
        // These two timestamps get set by the firmware when it is received
        request.rawStreamHeader.llNanoseconds = 0;
        request.rawStreamHeader.llArrivalNanoseconds = 0;
        request.rawStreamHeader.dwFrameId = frame_num;

        request.metadata.dwMetadataLength = 4;
        request.metadata.flTemperature = 0;
        request.metadata.dwFrameLength = 12;
        request.metadata.flVx = translational_velocity.x;
        request.metadata.flVy = translational_velocity.y;
        request.metadata.flVz = translational_velocity.z;

        _device->stream_write(&request.rawStreamHeader.header);

        return true;
    }

    sensor_temperature tm2_sensor::get_temperature(int sensor_id)
    {
        uint8_t buffer[BUFFER_SIZE];
        bulk_message_request_get_temperature request = {{ sizeof(request), DEV_GET_TEMPERATURE }};
        bulk_message_response_get_temperature* response = (bulk_message_response_get_temperature*)buffer;
        _device->bulk_request_response(request, *response, BUFFER_SIZE);

        if(uint32_t(sensor_id) > response->dwCount)
            throw wrong_api_call_sequence_exception("Requested temperature for an unknown sensor id");

        return response->temperature[sensor_id];
    }

    void tm2_sensor::set_exposure_and_gain(float exposure_ms, float gain)
    {
        bulk_message_request_set_exposure request = {{ sizeof(request), DEV_SET_EXPOSURE }};
        request.header.dwLength = offsetof(bulk_message_request_set_exposure, stream) + 2 * sizeof(stream_exposure);
        request.bNumOfVideoStreams = 2;
        for(int i = 0; i < 2; i++) {
            request.stream[i].bCameraID = SET_SENSOR_ID(SensorType::Fisheye, i);
            request.stream[i].dwIntegrationTime = exposure_ms; // TODO: you cannot set fractional ms, is this actually microseconds?
            request.stream[i].fGain = gain;
        }
        bulk_message_response_set_exposure response = {};

        _device->bulk_request_response(request, response);
    }

    void tm2_sensor::set_exposure(float value)
    {
        if (!manual_exposure)
            throw std::runtime_error("To control exposure you must set sensor to manual exposure mode prior to streaming");

        set_exposure_and_gain(value, last_gain);
        last_exposure = value;
    }

    float tm2_sensor::get_exposure() const
    {
        return last_exposure;
    }

    void tm2_sensor::set_gain(float value)
    {
        if (!manual_exposure)
            throw std::runtime_error("To control gain you must set sensor to manual exposure mode prior to streaming");

        set_exposure_and_gain(last_exposure, value);
        last_gain = value;
    }

    float tm2_sensor::get_gain() const
    {
        return last_gain;
    }

    void tm2_sensor::set_manual_exposure(bool manual)
    {
        if(_is_streaming)
            throw wrong_api_call_sequence_exception("Exposure mode cannot be controlled while streaming!");

        bulk_message_request_set_exposure_mode_control request = {{ sizeof(request), DEV_EXPOSURE_MODE_CONTROL }};
        request.bAntiFlickerMode = 0;
        request.bVideoStreamsMask = 0;
        if(!manual) {
            request.bVideoStreamsMask = 0x3; // bitstream 0011 to enable both fisheye autoexposure
        }

        bulk_message_response_set_exposure_mode_control response = {};
        _device->bulk_request_response(request, response);

        manual_exposure = manual;
    }

    ///////////////
    // Device

    tm2_device::tm2_device( std::shared_ptr< const tm2_info > const & dev_info ) :
        device( dev_info )
    {
        auto const & group = dev_info->get_group();
        // The mipi list did not exist when this check was written; a T265 is a single USB
        // node and nothing else, on any transport.
        if( group.usb_devices.size() != 1 || ! group.uvc_devices.empty()
            || ! group.hid_devices.empty() || ! group.mipi_devices.empty() )
            throw io_exception("Tried to create a T265 with a bad backend_device_group");

        LOG_DEBUG("Creating a T265 device");

        usb_device = platform::usb_enumerator::create_usb_device(group.usb_devices[0]);
        if(!usb_device)
            throw io_exception("Unable to create USB device");

        usb_info = usb_device->get_info();
        const std::vector<platform::rs_usb_interface> interfaces = usb_device->get_interfaces();
        for(auto & iface : interfaces) {
            auto endpoints = iface->get_endpoints();
            for(const auto & endpoint : endpoints) {
                int addr = endpoint->get_address();

                if      (addr == ENDPOINT_HOST_IN)  endpoint_bulk_in = endpoint;
                else if (addr == ENDPOINT_HOST_OUT) endpoint_bulk_out = endpoint;
                else if (addr == ENDPOINT_HOST_MSGS_IN)  endpoint_msg_in = endpoint;
                else if (addr == ENDPOINT_HOST_MSGS_OUT) endpoint_msg_out = endpoint;
                else if (addr == ENDPOINT_HOST_INT_IN)   endpoint_int_in = endpoint;
                else if (addr == ENDPOINT_HOST_INT_OUT)  endpoint_int_out = endpoint;
                else    LOG_ERROR("Unknown endpoint address " << addr);
            }
        }
        if(!endpoint_bulk_in || !endpoint_bulk_out || !endpoint_msg_in || !endpoint_msg_out || !endpoint_int_in || !endpoint_int_out)
            throw io_exception("Missing a T265 usb endpoint");


        usb_messenger = usb_device->open(0); // T265 only has one interface, 0
        if(!usb_messenger)
            throw io_exception("Unable to open device interface");

        LOG_DEBUG("Successfully opened and claimed interface 0");

        bulk_message_request_set_low_power_mode power_request = {{ sizeof(power_request), DEV_SET_LOW_POWER_MODE }};
        bulk_message_response_set_low_power_mode power_response = {};
        power_request.bEnabled = 0; // disable low power mode
        auto response = bulk_request_response(power_request, power_response);
        if(response != platform::RS2_USB_STATUS_SUCCESS)
            throw io_exception("Unable to communicate with device");

        bulk_message_request_get_device_info info_request = {{ sizeof(info_request), DEV_GET_DEVICE_INFO }};
        bulk_message_response_get_device_info info_response = {};
        bulk_request_response(info_request, info_response);

        if(info_response.message.bStatus == 0x1 || info_response.message.dwStatusCode == FW_STATUS_CODE_NO_CALIBRATION_DATA)
            throw io_exception("T265 device is uncalibrated");

        std::string serial = rsutils::string::from() << std::uppercase << std::hex << (bytesSwap(info_response.message.llSerialNumber) >> 16);
        LOG_INFO("Serial: " << serial);

        LOG_INFO("Connection type: " << platform::usb_spec_names.at(usb_info.conn_spec));

        register_info(RS2_CAMERA_INFO_NAME, tm2_device_name());
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, serial);
        std::string firmware = rsutils::string::from() << std::to_string(info_response.message.bFWVersionMajor) << "." << std::to_string(info_response.message.bFWVersionMinor) << "." << std::to_string(info_response.message.bFWVersionPatch) << "." << std::to_string(info_response.message.dwFWVersionBuild);
        register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION, firmware);
        LOG_INFO("Firmware version: " << firmware);
        {
            // librealsense::hexify() was removed from types.h; reproduce its formatting
            // (zero-padded, uppercase, two characters per byte) so the reported product ID
            // string is unchanged from what applications used to see.
            std::ostringstream oss;
            oss << std::setw( sizeof( usb_info.pid ) * 2 ) << std::setfill( '0' ) << std::uppercase
                << std::hex << usb_info.pid;
            register_info( RS2_CAMERA_INFO_PRODUCT_ID, oss.str() );
        }
        register_info(RS2_CAMERA_INFO_PRODUCT_LINE, "T200");

        register_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR, platform::usb_spec_names.at(usb_info.conn_spec));
        register_info(RS2_CAMERA_INFO_PHYSICAL_PORT, usb_info.id); // TODO uvc has device_path, usb_info has id and unique_id

        _sensor = std::make_shared<tm2_sensor>(this);
        add_sensor(_sensor);

        _sensor->register_option(rs2_option::RS2_OPTION_ASIC_TEMPERATURE, std::make_shared<asic_temperature_option>(*_sensor));
        _sensor->register_option(rs2_option::RS2_OPTION_MOTION_MODULE_TEMPERATURE, std::make_shared<motion_temperature_option>(*_sensor));

        _sensor->register_option(rs2_option::RS2_OPTION_EXPOSURE, std::make_shared<exposure_option>(*_sensor));
        _sensor->register_option(rs2_option::RS2_OPTION_GAIN, std::make_shared<gain_option>(*_sensor));
        _sensor->register_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, std::make_shared<exposure_mode_option>(*_sensor));

        _sensor->register_option(rs2_option::RS2_OPTION_ENABLE_MAPPING,             std::make_shared<tracking_mode_option<SIXDOF_MODE_ENABLE_MAPPING,              SIXDOF_MODE_NORMAL,         false>>(*_sensor, "Use an on device map (recommended)"));
        _sensor->register_option(rs2_option::RS2_OPTION_ENABLE_RELOCALIZATION,      std::make_shared<tracking_mode_option<SIXDOF_MODE_ENABLE_RELOCALIZATION,       SIXDOF_MODE_ENABLE_MAPPING, false>>(*_sensor, "Use appearance based relocalization (depends on mapping)"));
        _sensor->register_option(rs2_option::RS2_OPTION_ENABLE_POSE_JUMPING,        std::make_shared<tracking_mode_option<SIXDOF_MODE_DISABLE_JUMPING,             SIXDOF_MODE_ENABLE_MAPPING,  true>>(*_sensor, "Allow pose jumping (depends on mapping)"));
        _sensor->register_option(rs2_option::RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION, std::make_shared<tracking_mode_option<SIXDOF_MODE_DISABLE_DYNAMIC_CALIBRATION, SIXDOF_MODE_NORMAL,          true>>(*_sensor, "Enable dynamic calibration (recommended)"));
        _sensor->register_option(rs2_option::RS2_OPTION_ENABLE_MAP_PRESERVATION,    std::make_shared<tracking_mode_option<SIXDOF_MODE_ENABLE_MAP_PRESERVATION,     SIXDOF_MODE_ENABLE_MAPPING, false>>(*_sensor, "Preserve the map from the previous run as if it was loaded"));

        // Adding the extrinsic nodes to the default group
        auto tm2_profiles = _sensor->get_stream_profiles();
        for (auto && pf : tm2_profiles)
            register_stream_to_extrinsic_group(*pf, 0);

        //For manual testing: enable_loopback("C:\\dev\\recording\\tm2.bag");
    }

    tm2_device::~tm2_device()
    {
        // Since the device claims the interface and opens it, it
        // should also dispose of it
        LOG_DEBUG("Stopping sensor");
        // Note this is the last chance the tm2_sensor gets to use USB
        _sensor->dispose();
        LOG_DEBUG("Destroying T265 device");
    }

    void tm2_device::hardware_reset()
    {
        LOG_INFO("Sending hardware reset");
        uint32_t transferred;
        usb_messenger->control_transfer(0x40, 0x10, 0, 0, nullptr, 0, transferred, USB_TIMEOUT);
    }

    template<typename Request, typename Response>
    platform::usb_status tm2_device::bulk_request_response(const Request &request, Response &response, size_t max_response_size, bool assert_success)
    {
        std::lock_guard<std::mutex> lock(bulk_mutex);

        // request
        uint32_t length = request.header.dwLength;
        uint16_t message_id = request.header.wMessageID;
        LOG_DEBUG("Sending message " << message_name(request.header) << " length " << length);
        uint32_t transferred = 0;
        platform::usb_status e;
        e = usb_messenger->bulk_transfer(endpoint_msg_out, (uint8_t*)&request, length, transferred, USB_TIMEOUT);
        if (e != platform::RS2_USB_STATUS_SUCCESS) {
            LOG_ERROR("Bulk request error " << platform::usb_status_to_string.at(e));
            return e;
        }
        if (transferred != length) {
            LOG_ERROR("error: sent " << transferred << " not " << length);
            return platform::RS2_USB_STATUS_OTHER;
        }

        // response
        if(max_response_size == 0)
            max_response_size = sizeof(response);
        LOG_DEBUG("Receiving message with max_response_size " << max_response_size);

        transferred = 0;
        e = usb_messenger->bulk_transfer(endpoint_msg_in, (uint8_t*)&response, int(max_response_size), transferred, USB_TIMEOUT);
        if (e != platform::RS2_USB_STATUS_SUCCESS) {
            LOG_ERROR("Bulk response error " << platform::usb_status_to_string.at(e));
            return e;
        }
        if (transferred != response.header.dwLength) {
            LOG_ERROR("Received " << transferred << " but header was " << response.header.dwLength << " bytes (max_response_size was " << max_response_size << ")");
            return platform::RS2_USB_STATUS_OTHER;
        }
        if (assert_success && MESSAGE_STATUS(response.header.wStatus) != MESSAGE_STATUS::SUCCESS) {
            LOG_ERROR("Received " << message_name(response.header) << " with length " << response.header.dwLength << " but got non-zero status of " << status_name(response.header));
        }
        LOG_DEBUG("Received " << message_name(response.header) << " with length " << response.header.dwLength);
        return e;
    }

    // all messages must have dwLength and wMessageID as first two members
    platform::usb_status tm2_device::stream_write(const t265::bulk_message_request_header * request)
    {
        std::lock_guard<std::mutex> lock(stream_mutex);

        uint32_t length = request->dwLength;
        uint16_t message_id = request->wMessageID;
        LOG_DEBUG("Sending stream message " << message_name(*request) << " length " << length);
        uint32_t transferred = 0;
        platform::usb_status e;
        e = usb_messenger->bulk_transfer(endpoint_bulk_out, (uint8_t *)request, length, transferred, USB_TIMEOUT);
        if (e != platform::RS2_USB_STATUS_SUCCESS) {
            LOG_ERROR("Stream write error " << platform::usb_status_to_string.at(e));
            return e;
        }
        if (transferred != length) {
            LOG_ERROR("error: sent " << transferred << " not " << length);
            return platform::RS2_USB_STATUS_OTHER;
        }
        return e;
    }

    platform::rs_usb_request tm2_device::stream_read_request(std::vector<uint8_t> & buffer, std::shared_ptr<platform::usb_request_callback> callback)
    {
        auto request = usb_messenger->create_request(endpoint_bulk_in);
        request->set_buffer(buffer);
        request->set_callback(callback);
        return request;
    }

    void tm2_device::submit_request(platform::rs_usb_request request)
    {
        auto e = usb_messenger->submit_request(request);
        if (e != platform::RS2_USB_STATUS_SUCCESS)
            throw std::runtime_error("failed to submit request, error: " + platform::usb_status_to_string.at(e));
    }

    bool tm2_device::cancel_request(platform::rs_usb_request request)
    {
        auto e = usb_messenger->cancel_request(request);
        if (e != platform::RS2_USB_STATUS_SUCCESS)
            return false;
        return true;
    }

    platform::rs_usb_request tm2_device::interrupt_read_request(std::vector<uint8_t> & buffer, std::shared_ptr<platform::usb_request_callback> callback)
    {
        auto request = usb_messenger->create_request(endpoint_int_in);
        request->set_buffer(buffer);
        request->set_callback(callback);
        return request;
    }

    /**
    * Enable loopback will replace the input and ouput of the tm2 sensor
    */
    void tm2_device::enable_loopback(const std::string& source_file)
    {
        auto ctx = get_context();
        std::shared_ptr<playback_device> raw_streams;
        try
        {
            raw_streams = std::make_shared<playback_device>( std::make_shared<playback_device_info>( ctx, source_file ),
                                                             std::make_shared<ros_reader>( source_file, ctx ) );
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("Failed to create playback device from given file. File = \"" << source_file << "\". Exception: " << e.what());
            throw librealsense::invalid_value_exception("Failed to enable loopback");
        }
        _sensor->enable_loopback(raw_streams);
        update_info(RS2_CAMERA_INFO_NAME, rsutils::string::from() << tm2_device_name() << " (Loopback - " << source_file << ")");
    }

    void tm2_device::disable_loopback()
    {
        _sensor->disable_loopback();
        update_info(RS2_CAMERA_INFO_NAME, tm2_device_name());
    }

    bool tm2_device::is_enabled() const
    {
        return _sensor->is_loopback_enabled();
    }

    void tm2_device::register_stream_to_extrinsic_group(const stream_interface& stream, uint32_t group_index)
    {
        //T265 uses POSE sensor as reference sensor for extrinsics
        auto tm2_profiles = _sensor->get_stream_profiles();
        int ref_index = 0;
        for (int i = 0; i < tm2_profiles.size(); i++)
            if (tm2_profiles[i]->get_stream_type() == RS2_STREAM_POSE)
            {
                ref_index = i;
                break;
            }

        _extrinsics[stream.get_unique_id()] = std::make_pair(group_index, tm2_profiles[ref_index]);
    }


    // ==========================================================================================
    // Host-side stereo depth
    //
    // The device cannot produce depth, so it is computed here from the rectified fisheye pair.
    // Matching takes a few hundred milliseconds, so it runs on a worker thread that always
    // takes the newest complete pair and discards anything that arrived while it was busy.
    // Depth therefore runs at a lower and more variable rate than the fisheye streams, which is
    // the intended trade: dropping frames is far better than stalling the USB message loop.

    void tm2_sensor::build_stereo_maps()
    {
        if( _stereo_maps_ready )
            return;

        std::shared_ptr< stream_profile_interface > left, right;
        for( auto const & p : initialized_profiles() )
        {
            if( p->get_stream_type() != RS2_STREAM_FISHEYE )
                continue;
            if( p->get_stream_index() == 1 ) left = p;
            if( p->get_stream_index() == 2 ) right = p;
        }
        if( ! left || ! right )
        {
            LOG_ERROR( "Cannot build stereo maps: fisheye profiles not found" );
            return;
        }

        auto Kl = get_intrinsics( to_profile( left.get() ) );
        auto Kr = get_intrinsics( to_profile( right.get() ) );

        // Extrinsics here are held relative to the pose stream, so compose left->pose->right:
        // R = Rr^T * Rl and t = Rr^T * (tl - tr). rs2_extrinsics stores column-major.
        auto El = get_extrinsics( *left, 0 );
        auto Er = get_extrinsics( *right, 0 );

        float R_col[9], t[3];
        for( int r = 0; r < 3; ++r )
            for( int c = 0; c < 3; ++c )
            {
                float sum = 0.f;
                for( int k = 0; k < 3; ++k )
                    sum += Er.rotation[r * 3 + k] * El.rotation[c * 3 + k];
                R_col[c * 3 + r] = sum;
            }
        for( int r = 0; r < 3; ++r )
        {
            float sum = 0.f;
            for( int k = 0; k < 3; ++k )
                sum += Er.rotation[r * 3 + k] * ( El.translation[k] - Er.translation[k] );
            t[r] = sum;
        }

        _stereo_baseline = std::sqrt( t[0] * t[0] + t[1] * t[1] + t[2] * t[2] );

        // build_remap wants row-major.
        float R_row[9];
        for( int r = 0; r < 3; ++r )
            for( int c = 0; c < 3; ++c )
                R_row[r * 3 + c] = R_col[c * 3 + r];

        float const identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
        _remap_left  = t265_stereo::build_remap( Kl, identity, _stereo_cfg );
        _remap_right = t265_stereo::build_remap( Kr, R_row,    _stereo_cfg );

        _matcher_cfg.max_disparity = _stereo_cfg.max_disparity;

        int const W = _stereo_cfg.width_px(), H = _stereo_cfg.height_px;
        _pending_left.assign( size_t( W ) * H, 0 );
        _pending_right.assign( size_t( W ) * H, 0 );
        _have_left = _have_right = false;

        _stereo_maps_ready = true;
        LOG_INFO( "T265 stereo depth ready: baseline " << _stereo_baseline << "m, output "
                  << ( W - _stereo_cfg.max_disparity ) << "x" << H );
    }

    void tm2_sensor::start_stereo_worker()
    {
        build_stereo_maps();
        if( ! _stereo_maps_ready )
            return;

        _stereo_stop = false;
        _stereo_thread = std::thread( [this]() { stereo_worker(); } );
    }

    void tm2_sensor::stop_stereo_worker()
    {
        if( ! _stereo_thread.joinable() )
            return;

        {
            std::lock_guard< std::mutex > lk( _stereo_mutex );
            _stereo_stop = true;
        }
        _stereo_cv.notify_all();
        _stereo_thread.join();

        std::lock_guard< std::mutex > lk( _stereo_mutex );
        _have_left = _have_right = false;
    }

    void tm2_sensor::submit_fisheye_for_depth( int sensor_id, uint8_t const * data,
                                               int width, int height, double timestamp,
                                               unsigned long long frame_number )
    {
        if( ! _stereo_maps_ready || ! data )
            return;

        auto const & table = ( sensor_id == 0 ) ? _remap_left : _remap_right;

        std::lock_guard< std::mutex > lk( _stereo_mutex );
        auto & dst = ( sensor_id == 0 ) ? _pending_left : _pending_right;

        // Rectify here rather than in the worker: it costs about 2ms and means only the
        // small rectified image is retained rather than a copy of the full 848x800 source.
        t265_stereo::remap_y8( data, width, height, table, dst.data() );

        if( sensor_id == 0 ) _have_left = true;
        else                 _have_right = true;

        if( _have_left && _have_right )
        {
            _pending_timestamp = timestamp;
            _pending_frame_number = frame_number;
            _stereo_cv.notify_one();
        }
    }

    void tm2_sensor::stereo_worker()
    {
        int const W = _stereo_cfg.width_px(), H = _stereo_cfg.height_px;
        int const roi_w = W - _stereo_cfg.max_disparity;
        float const focal = _stereo_cfg.focal_px();
        float const depth_units = 0.001f;

        std::vector< uint8_t >  left( size_t( W ) * H ), right( size_t( W ) * H );
        std::vector< float >    disparity( size_t( W ) * H );
        std::vector< uint16_t > depth( size_t( W ) * H );

        while( true )
        {
            double ts = 0.0;
            unsigned long long fn = 0;
            {
                std::unique_lock< std::mutex > lk( _stereo_mutex );
                _stereo_cv.wait( lk, [this]() { return _stereo_stop || ( _have_left && _have_right ); } );
                if( _stereo_stop )
                    return;

                // Take the pair and clear the slots, so frames arriving while we compute
                // replace each other rather than queueing up.
                left  = _pending_left;
                right = _pending_right;
                ts = _pending_timestamp;
                fn = _pending_frame_number;
                _have_left = _have_right = false;
            }

            t265_stereo::match_disparity( left.data(), right.data(), W, H, _matcher_cfg,
                                          disparity.data() );
            t265_stereo::disparity_to_depth_z16( disparity.data(), W, H, focal, _stereo_baseline,
                                                 depth_units, 0.15f, 8.f, depth.data() );

            if( _stereo_stop || ! _is_streaming || ! _depth_profile )
                continue;

            frame_additional_data additional_data( ts, fn, ts, 0, nullptr, ts, 0, 0, false, 0.0 );

            frame_holder frame = _source.alloc_frame(
                { RS2_STREAM_DEPTH, 0, RS2_EXTENSION_DEPTH_FRAME },
                size_t( roi_w ) * H * sizeof( uint16_t ),
                std::move( additional_data ),
                true );

            if( ! frame.frame )
            {
                LOG_DEBUG( "Dropped depth frame: no buffer available" );
                continue;
            }

            auto video = static_cast< video_frame * >( frame.frame );
            video->assign( roi_w, H, roi_w * int( sizeof( uint16_t ) ), 16 );
            frame->set_timestamp( ts );
            frame->set_timestamp_domain( RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK );
            frame->set_stream( _depth_profile );

            // Copy out the region of interest, dropping the disparity search margin.
            auto * out = reinterpret_cast< uint16_t * >(
                const_cast< uint8_t * >( video->get_frame_data() ) );
            for( int y = 0; y < H; ++y )
                std::memcpy( out + size_t( y ) * roi_w,
                             depth.data() + size_t( y ) * W + _stereo_cfg.max_disparity,
                             size_t( roi_w ) * sizeof( uint16_t ) );

            _source.invoke_callback( std::move( frame ) );
        }
    }

}
