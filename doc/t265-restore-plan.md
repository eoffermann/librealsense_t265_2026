# T265 Restoration Plan

**Status:** Draft — not yet started
**Date:** 2026-07-18
**Baseline:** `development` @ `21a206d27`
**Also verified against:** `master` @ `b9d9454b8` (identical results — see §1)

## Goal

Restore Intel RealSense T265 (TM2) tracking-camera support to this fork, and keep it
working as upstream librealsense continues to evolve.

---

## 1. Strategy: forward-port, do not back-port

The instinctive approach — keep a T265-era base and replay upstream changes over it — is
not viable. Divergence since the removal (`391d5356e`):

| Measure | vs `development` | vs `master` |
|---|---|---|
| Commits | 8,712 | 7,779 |
| Files changed | 3,387 | 3,312 |
| Line delta | +417,709 / −561,961 | +402,758 / −560,294 |

**Instead: port T265 forward onto current `development`.**

This was validated empirically. Reverting the removal merge (`git revert -m 1 1bcaa97fe`)
onto **both** branches yields an identical result:

- **10 conflicted files** (out of 33 touched)
- **All 8 `src/tm2/*` files restore cleanly** — they were pure deletions, so nothing conflicts

That the conflict set is identical across two branches 935 commits apart is a useful signal:
the integration surface is stable, and the estimate is unlikely to drift as upstream moves.

Conflicting files: `.github/workflows/buildsCI.yaml`, `CMakeLists.txt`,
`common/fw/CMakeLists.txt`, `common/fw/fw.rc`, `common/model-views.cpp`,
`common/viewer.cpp`, `src/backend.h`, `src/context.cpp`, `src/rs.cpp`,
`unit-tests/unit-tests-live.cpp`.

Note that several conflicts are *spurious* — upstream relocated code that the revert wants
to reinsert (`src/backend.h` is the clearest case). Resolve by keeping `HEAD` and re-adding
only the T265-specific hunks.

## 2. Relevant history

| Commit | Date | Description |
|---|---|---|
| `391d5356e` | 2023-01-02 | Last commit with full T265 support |
| `1bcaa97fe` | 2023-01-04 | PR #11287 — Remove TM2 (T265) from LibRS (merge; use `-m 1`) |
| `fe5aff43f` | 2023-01-24 | Remove T265-related examples |
| `b6ee94d4d` | 2023-01-24 | Remove T265-related examples from `wrappers/` |
| `1cde72625` | 2023-01-24 | T265 leftovers in docs and src |
| `e6486198e` | 2023-02-27 | Fix crash from leftover T265 |
| `d1075a84c` | (later) | PR #15063 — Remove D400 bundled FW (deleted `common/fw/`) |

Source to recover (4,201 lines total, from `391d5356e`):

```
  2170  src/tm2/tm-device.cpp      <- the bulk of the work
  1532  src/tm2/t265-messages.h    <- pure protocol, ports as-is
   219  src/tm2/tm-device.h
   117  src/tm2/message-print.h
    61  src/tm2/tm-info.cpp        <- must be rewritten (see Phase 2)
    54  src/tm2/tm-boot.h
    27  src/tm2/tm-info.h
    21  src/tm2/CMakeLists.txt
```

## 3. What still exists on master

Substantially more survives than expected — the pose *frame* pipeline is intact and
actively maintained; only the *device/sensor* layer was removed.

| Component | State | Reference |
|---|---|---|
| `RS2_STREAM_POSE` | Present | `include/librealsense2/h/rs_sensor.h:57` |
| `RS2_FORMAT_6DOF` | Present | `include/librealsense2/h/rs_sensor.h:89` |
| `rs2_pose` struct | Present | `include/librealsense2/h/rs_types.h:110` |
| `rs2_pose_frame_get_pose_data` | Fully implemented | `src/rs.cpp:3117` |
| `librealsense::pose_frame` | Present, header dated **2024** | `src/core/pose-frame.h:14` |
| Frame archive pose allocation | Working | `src/archive.cpp:42` |
| `rs2::pose_frame`, `pose_sensor` | Full API surface | `include/librealsense2/hpp/` |
| Python bindings | Fully present | `wrappers/python/` |
| Legacy rosbag record/playback | Full round-trip | `src/media/ros/` |
| `RS2_PRODUCT_LINE_T200` | Still defined | `include/librealsense2/h/rs_context.h:135` |
| T265 udev rules | Still present | `config/99-realsense-libusb.rules:55-56` |
| `pose_sensor_interface` | **Declared, zero implementers** | `src/core/motion.h:27` |

The `pose-frame.h` copyright year (2024) is significant: pose frame handling was maintained
*after* the T265 removal, so this is not bit-rotted code.

**Firmware availability — verified 2026-07-18:**

```
https://librealsense.intel.com/Releases/TM2/FW/target/0.2.0.951/target-0.2.0.951.mvcmd
  -> HTTP 200, 9,323,648 bytes
  SHA1: c3940ccbb0e3045603e4aceaa2d73427f96e24bc
```

This was the single largest feasibility risk (T265 is EOL). It is currently fine, but the
blob **should be mirrored locally now** — do not depend on Intel keeping it up.

## 4. What actually changed underneath

Three upstream changes make this a port rather than a revert:

1. **Device enumeration was rearchitected.** `device_info` now uses a no-arg
   `create_device()`; the old `create(ctx, register_device_notifications)` and
   `get_device_data()` are gone. Discovery flows through
   `backend_device_factory::create_devices_from_group`. `tm-info.cpp` must be rewritten.

2. **Bundled firmware was removed entirely** (PR #15063). `common/fw/` no longer exists, and
   `master` has no firmware-fetch mechanism at all. T265 requires firmware pushed over USB at
   boot, so a delivery path must be reintroduced.

3. **`unit-tests/unit-tests-live.cpp` was deleted** — tests migrated to pytest.

The USB abstraction layer (`src/usb/`) is unchanged in the ways that matter:
`usb_enumerator::create_usb_device()` and `query_devices_info()` retain their signatures, so
`tm-boot.h` should port with little or no change.

---

## 5. Phases

### Phase 0 — Branch topology

Create `t265-restore` off `origin/development` (the project's base branch, and where upstream
changes land first). Keep tracking branches pristine; never commit T265 code to them. All
T265 work lives on the feature branch, with `development` merged forward periodically. This
is what makes Phase 9 cheap.

**Note on remotes:** `origin` currently points at the *upstream* repository
(`realsenseai/librealsense`). There is no `fork` remote configured. T265 restoration will not
be accepted upstream — it was removed deliberately — so this work must never be pushed to
`origin`. Configure a `fork` remote before any push.

### Phase 1 — Mechanical restore

Revert the four removal commits onto the branch:

```bash
git revert --no-commit -m 1 1bcaa97fe
git revert --no-commit fe5aff43f b6ee94d4d 1cde72625
```

Resolve the 10 conflicts by keeping `HEAD` and re-adding only T265-specific hunks. Expect
`src/tm2/*` to land untouched.

**Estimate:** 1–2 days. Cost is well characterized.

### Phase 2 — Re-architect device enumeration

Rewrite `tm2_info` to derive `platform::platform_device_info`, using its USB-only group
constructor (`src/platform/backend-device-group.h:80`). That inherits `get_address()`,
`to_stream()` and leaves `create_device()` as the only pure virtual:

```cpp
// src/tm2/tm-info.h
class tm2_info : public platform::platform_device_info
{
    typedef platform::platform_device_info super;
public:
    explicit tm2_info( std::shared_ptr< context > const & ctx, platform::usb_device_info const & hwm )
        : platform_device_info( ctx, { { hwm } } ) {}

    std::shared_ptr< device_interface > create_device() override;

    static std::vector< std::shared_ptr< tm2_info > >
        pick_tm2_devices( std::shared_ptr< context > ctx,
                          std::vector< platform::usb_device_info > & usb );
};
```

Hook into `src/backend-device-factory.cpp`, after the D500 block (~line 209):

```cpp
#ifdef BUILD_WITH_TM2
if( mask & RS2_PRODUCT_LINE_T200 ) {
    auto tm2_devices = tm2_info::pick_tm2_devices( ctx, devices.usb_devices );
    std::copy( begin(tm2_devices), end(tm2_devices), std::back_inserter(list) );
}
#endif
```

Closest existing template: `fw_update_info` (`src/fw-update/fw-update-factory.h:12`) — the
only other USB-only `device_info` in the tree.

> **Must override `is_same_as`.** `backend_device_group::operator==`
> (`src/platform/backend-device-group.h:97`) compares only `uvc_devices` and `hid_devices`,
> ignoring `usb_devices`. Inheriting `platform_device_info::is_same_as` would make every
> T265 compare identical to every other T265 *and* to every recovery device. Override it to
> compare `_group.usb_devices` explicitly. `fw_update_info` avoids this only because hotplug
> diffing uses `is_contained_in`, which does check USB — but `is_same_as` is used elsewhere
> (`device_hub`, `rs2_device_info` comparison).

**Estimate:** ~60 lines. Small.

### Phase 3 — Port `tm_device` — **the bulk of the work**

Port 2,170 lines of `tm-device.cpp` onto the modern `backend_device` + sensor base classes.
Note the dual-base virtual-inheritance constructor pattern
(`src/platform-camera.cpp:104-108`) — both `device` and `backend_device` must be initialized.

`t265-messages.h` (1,532 lines) is pure wire protocol with no SDK dependencies and should
port as-is.

**This phase implements `pose_sensor_interface`** — the missing piece that currently makes
`rs2::pose_sensor` throw for every device.

**This is the main risk and the main unknown.** Timebox a spike here before committing to
any schedule.

### Phase 4 — Firmware delivery

Upstream deliberately deleted the bundling machinery, so do not resurrect `common/fw/`.

**Recommended:** load `target-0.2.0.951.mvcmd` from disk at runtime via a configurable path,
with CMake optionally fetching and verifying it (SHA1 above). Rationale:

- Avoids re-adding a subsystem upstream removed — which would conflict on every future merge
- Decouples the build from an EOL download URL
- Keeps the firmware blob out of git history

Mirror the blob to internal storage regardless of which option is chosen.

### Phase 5 — Restore API surface

- `include/librealsense2/hpp/rs_device.hpp` — restore `pose_sensor`, `wheel_odometer`, `tm2`
  classes (−134 lines in the removal)
- `src/rs.cpp` — un-stub `rs2_connect_tm2_controller` / `rs2_disconnect_tm2_controller`
  (currently `throw not_implemented_exception("deprecated")` at `:3440`, `:3446`) and
  `rs2_context_unload_tracking_module` (`:2137`, empty no-op)
- `src/rs.cpp:2000`, `:1959` — `RS2_EXTENSION_TM2` / `RS2_EXTENSION_TM2_SENSOR` are hardcoded
  `return false`; restore real dispatch
- `wrappers/python/pyrs_device.cpp` — re-add bindings (−17 lines)

ABI note: `src/realsense.def` still exports the TM2 symbols, so the ABI slot is intact.

### Phase 6 — Fix two latent bugs found during analysis

1. **`src/source.cpp:215`** — `frame_source::stream_to_frame_types()` maps `RS2_STREAM_POSE`
   into the `RS2_EXTENSION_VIDEO_FRAME` group. Any pose stream routed through this helper
   allocates a video frame and fails the `POSE_FRAME` extension check. Callers:
   `src/uvc-sensor.cpp:199`, `src/media/ros/ros_reader.cpp:442`,
   `src/media/ros2/ros2_reader.cpp:528`. The legacy rosbag reader dodges it by hardcoding the
   extension (`ros_reader.cpp:602`); a new pose-capable sensor would not.

2. **`src/media/ros2/ros2_writer.cpp:534`** — the `RS2_EXTENSION_POSE_PROFILE` case is
   commented out, so ros2 recordings cannot carry pose profiles. Restore only if ros2 bags
   are needed; the legacy rosbag path is unaffected.

### Phase 7 — Viewer, tools, examples

Restore the T265 3D model, pose rendering, and trajectory/pose examples. Lowest risk and
fully deferrable — do not let this block Phase 8.

### Phase 8 — Build and hardware validation

- Reinstate the `BUILD_WITH_TM2` option in `CMake/lrs_options.cmake` and the `add_tm2()`
  macro (`libusb` link + `WITH_TRACKING=1` define)
- Restore `include(${_rel_path}/tm2/CMakeLists.txt)` in `src/CMakeLists.txt`
- CI wiring in `.github/workflows/buildsCI.yaml`

Then validate on real hardware: enumerate → boot (firmware push) → stream pose + fisheye →
record and play back.

**Nothing before this proves the port works.** The plan is unverified until hardware streams.

### Phase 9 — Ongoing upstream tracking

Merge `development` → `t265-restore` regularly (and on each upstream release). T265 code is
nearly all confined to `src/tm2/`, with only a handful of small integration hooks elsewhere,
so routine merges should be near-trivial.

Exposure to watch:

- `src/context.cpp` — 172 commits of churn since the removal
- `src/backend-device-factory.cpp` — the enumeration hook
- `src/core/motion.h` — the pose/odometry interfaces
- `src/source.cpp` — if the Phase 6 fix is not upstreamed

---

## 6. Risks

| Risk | Severity | Notes |
|---|---|---|
| Phase 3 effort is unbounded | **High** | 2,170 lines against rearchitected bases; spike before scheduling |
| No hardware CI | **High** | Upstream will never test this path; silent breakage on merge |
| Firmware URL retirement | Medium | Currently live; mirror the blob now |
| `libusb` / `WITH_TRACKING` regressions | Medium | Verify across target platforms |
| `is_same_as` collision (Phase 2) | Medium | Known and specified above; must not be missed |
| Viewer/examples drift | Low | Deferrable |

## 7. Working agreement

**Commit frequently.** Each phase lands as a series of small, self-describing commits rather
than one large drop. Per project convention: short one-sentence message, no username prefix
on branches, no `Co-Authored-By` trailer, plain `git commit -m "message"`. Restoring ~4,200
lines of driver code in one commit would make any later bisect useless — and bisect is the
main tool available when a merge from upstream silently breaks tracking.

**Verify before claiming.** A commit that restores code is not a commit that restores
functionality. State plainly which of these each change has reached:

| Level | Meaning |
|---|---|
| Compiles | Builds with `BUILD_WITH_TM2=ON` |
| Enumerates | Device appears in `rs-enumerate-devices` |
| Boots | Firmware pushed, device re-enumerates as `8087:0b37` |
| Streams | Pose and fisheye frames arrive with sane values |
| Round-trips | Record and playback reproduce pose data |

Phases 1–7 can reach *Compiles* with no hardware. **Nothing above that is provable without a
physical T265.** Until a device streams, this restoration is unverified regardless of how
much code has been committed — that gap should be stated in commit messages and status
updates, not glossed.

Do not mark a phase complete on the strength of a clean build alone.

## 8. Open questions

- Which platforms must be supported? (Linux-only would cut significant work)
- Are ros2 bags required, or is the legacy rosbag path sufficient?
- Is the T265 firmware blob mirrored internally yet?
