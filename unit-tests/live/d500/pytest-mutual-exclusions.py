# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

# Mutual exclusion between the inference stream and embedded filters.
#
# The decimation and temporal embedded filters cannot run at the same time as the
# inference stream, so the SDK rejects the conflicting combination in either order with
# a user-friendly (wrong-api-call-sequence) error, surfaced as a RuntimeError through
# the Python API. Other ("non-blocking") embedded filters, e.g. improved close range,
# are not restricted and can run alongside inference.
#
# The test adapts to the device at runtime: it skips when the inference stream or a
# given embedded filter is not present, rather than gating on firmware versions.

import pytest
import pyrealsense2 as rs
from pytest_check import check
import logging
log = logging.getLogger(__name__)


pytestmark = [
    pytest.mark.device_each( "D555", "D585" ),
    pytest.mark.context( "nightly" ),
]


# Embedded filters that are mutually exclusive with the inference stream.
BLOCKING_FILTERS = [
    rs.embedded_filter_type.decimation,
    rs.embedded_filter_type.temporal,
]

# Embedded filters that may run alongside the inference stream.
NON_BLOCKING_FILTERS = [
    rs.embedded_filter_type.improved_close_range_depth,
]

ENABLED = rs.option.embedded_filter_enabled


@pytest.fixture
def depth_sensor(test_device):
    dev, _ = test_device
    return dev.first_depth_sensor()


@pytest.fixture
def inference_sensor(test_device):
    dev, _ = test_device
    try:
        return dev.first_inference_sensor()  # throws if the device has no inference sensor
    except RuntimeError:
        pytest.skip( "Device has no inference sensor" )


@pytest.fixture
def inference_profile(inference_sensor):
    profile = next( ( p for p in inference_sensor.get_stream_profiles()
                      if p.stream_type() == rs.stream.object_detection ), None )
    if profile is None:
        pytest.skip( "Inference sensor has no object-detection profile" )
    return profile


def get_embedded_filter_or_skip(depth_sensor, filter_type):
    # get_embedded_filter throws (rather than returning falsy) when the filter is not present.
    try:
        return depth_sensor.get_embedded_filter( filter_type )
    except Exception:
        pytest.skip( f"Embedded filter {filter_type} not present on this device" )


def require_inference_openable(inference_sensor, inference_profile):
    # The rejection tests are only meaningful if the inference stream opens standalone;
    # otherwise we cannot tell our guard apart from an unrelated open failure.
    try:
        inference_sensor.open( inference_profile )
        inference_sensor.close()
    except RuntimeError as e:
        pytest.skip( f"Inference stream not openable on this device: {e}" )


def require_inference_started(inference_sensor, inference_profile):
    # An inference sensor that exists must be startable - a failure here is a real error, not a skip.
    inference_sensor.open( inference_profile )
    try:
        inference_sensor.start( lambda f: None )
    except Exception:
        inference_sensor.close()  # start never took effect; undo the open before propagating
        raise


@pytest.mark.parametrize( "filter_type", BLOCKING_FILTERS )
def test_inference_rejected_when_blocking_filter_enabled(depth_sensor, inference_sensor, inference_profile, filter_type):
    require_inference_openable( inference_sensor, inference_profile )
    embedded_filter = get_embedded_filter_or_skip( depth_sensor, filter_type )

    embedded_filter.set_option( ENABLED, 1.0 )
    try:
        # The guard runs before the device is touched, so opening must fail here.
        with pytest.raises( RuntimeError ):
            inference_sensor.open( inference_profile )
    finally:
        embedded_filter.set_option( ENABLED, 0.0 )

    # With the filter disabled the same open succeeds again - proving the filter was the cause.
    inference_sensor.open( inference_profile )
    inference_sensor.close()


@pytest.mark.parametrize( "filter_type", BLOCKING_FILTERS )
def test_blocking_filter_rejected_when_inference_active(depth_sensor, inference_sensor, inference_profile, filter_type):
    embedded_filter = get_embedded_filter_or_skip( depth_sensor, filter_type )
    require_inference_started( inference_sensor, inference_profile )
    try:
        # Enabling the filter while inference streams is rejected, and the value stays off.
        with pytest.raises( RuntimeError ):
            embedded_filter.set_option( ENABLED, 1.0 )
        check.equal( embedded_filter.get_option( ENABLED ), 0.0 )
    finally:
        inference_sensor.stop()
        inference_sensor.close()

    # Once inference stops, enabling the filter is allowed again.
    embedded_filter.set_option( ENABLED, 1.0 )
    embedded_filter.set_option( ENABLED, 0.0 )


@pytest.mark.parametrize( "filter_type", BLOCKING_FILTERS )
def test_blocking_filter_rejected_when_inference_opened_not_streaming(depth_sensor, inference_sensor,
                                                                      inference_profile, filter_type):
    # Covers the open()->start() window: the inference sensor is opened but not yet streaming. Enabling a blocking
    # filter in that window must still be rejected, otherwise the subsequent start() would leave both active.
    embedded_filter = get_embedded_filter_or_skip( depth_sensor, filter_type )
    try:
        inference_sensor.open( inference_profile )  # opened, not started
    except RuntimeError as e:
        pytest.skip( f"Inference stream not openable on this device: {e}" )
    try:
        with pytest.raises( RuntimeError ):
            embedded_filter.set_option( ENABLED, 1.0 )
        check.equal( embedded_filter.get_option( ENABLED ), 0.0 )
    finally:
        inference_sensor.close()


@pytest.mark.parametrize( "filter_type", NON_BLOCKING_FILTERS )
def test_non_blocking_filter_and_inference_coexist(depth_sensor, inference_sensor, inference_profile, filter_type):
    embedded_filter = get_embedded_filter_or_skip( depth_sensor, filter_type )
    initial = embedded_filter.get_option( ENABLED )

    embedded_filter.set_option( ENABLED, 1.0 )
    try:
        # Inference can start while the non-blocking filter is enabled...
        require_inference_started( inference_sensor, inference_profile )
        try:
            # ...and the non-blocking filter can be toggled while inference streams.
            embedded_filter.set_option( ENABLED, 0.0 )
            embedded_filter.set_option( ENABLED, 1.0 )
        finally:
            inference_sensor.stop()
            inference_sensor.close()
    finally:
        embedded_filter.set_option( ENABLED, initial )
