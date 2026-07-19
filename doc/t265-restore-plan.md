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

**Firmware availability — downloaded and verified 2026-07-18:**

```
https://librealsense.intel.com/Releases/TM2/FW/target/0.2.0.951/target-0.2.0.951.mvcmd
  -> HTTP 200, 9,323,648 bytes   (matches expected size)
  -> SHA1 c3940ccbb0e3045603e4aceaa2d73427f96e24bc   (matches the value the 2023
     build system checked against — the blob is bit-identical, not merely present)
```

This was the single largest feasibility risk (T265 is EOL). The full blob was fetched and
hash-verified, not just probed — so the firmware Intel serves today is confirmed good.

**Still outstanding:** the blob needs a permanent mirror. It currently exists only in a
session-scoped scratch directory, which will be lost. Do not treat this risk as closed until
it is stored somewhere durable and referenced from the build.

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

## 4a. Development environment — **RESOLVED (one caveat)**

Target dev/test machine is the Windows 11 laptop the T265 will be attached to. Audited
2026-07-18 and found to have no C++ toolchain at all; installed the same day.

| Requirement | State | Notes |
|---|---|---|
| CMake | ✅ **4.4.0** | User-scope winget install. See CMake 4.x note below |
| MSVC | ✅ **VS Build Tools 2022** 17.14.37502, toolset 14.44.35207 | `cl.exe` verified present |
| Python 3 | ✅ **3.12.10** | At `%LOCALAPPDATA%\Programs\Python\Python312\`. The Store `WindowsApps` alias still shadows it on PATH — disable the alias or use the full path |
| Git | ✅ 2.55.0 | — |
| **ATL (`atlcomcli.h`)** | ❌ **MISSING** | Required by the Media Foundation backend (`src/mf/`). See below |

### Baseline build achieved — 2026-07-18

`realsense2.dll` (8.2 MB) builds cleanly from `t265-restore` (which differs from
`origin/development` by documentation only, so it is a valid baseline):

```
cmake -S . -B build-rsusb -G "Visual Studio 17 2022" -A x64 \
      -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=OFF \
      -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF
cmake --build build-rsusb --config Release --parallel --target realsense2
  -> exit 0, zero errors
```

This is the known-good reference. Any build failure after Phase 1 is now attributable to the
port rather than the environment.

### Outstanding: ATL, blocked on a pending reboot

The default (Media Foundation) backend build fails with:

```
src/mf/mf-uvc.h(11,10): fatal error C1083: Cannot open include file: 'atlcomcli.h'
src/mf/mf-hid.h(10,10): fatal error C1083: Cannot open include file: 'atlcomcli.h'
```

`Microsoft.VisualStudio.Component.VC.ATL` was not pulled in by the VCTools workload's
`--includeRecommended`. Attempts to add it fail with VS installer error **8006** because a
**reboot is pending** from the Build Tools install (confirmed via `RebootPending` /
`RebootRequired` registry keys).

**Action required: reboot, then run**

```powershell
winget install --id Microsoft.VisualStudio.2022.BuildTools --force `
  --override "--quiet --wait --norestart --add Microsoft.VisualStudio.Component.VC.ATL"
```

This is not blocking Phase 1. `FORCE_RSUSB_BACKEND=ON` bypasses `src/mf/` entirely, and that
is the libusb/WinUSB path T265 uses anyway — so the T265 work can proceed on the RSUSB build
while the MF backend remains unbuildable. It **will** need fixing before any claim about
not regressing D400-class cameras on the default Windows backend.

### CMake 4.x note

CMake 4.x removed compatibility with projects declaring `cmake_minimum_required` below 3.5.
Every in-tree declaration is ≥ 3.8, and **configure succeeds** with only deprecation warnings.
Externally fetched dependencies (pybind11, Catch2, FastDDS) are the residual risk, and they
are pulled in only when Python bindings, unit tests, or DDS are enabled — none of which the
baseline exercised yet. If it bites, pin CMake 3.31.x.

**Windows-specific notes for T265:**

- T265 is a **raw USB device, not UVC**, so it goes through the WinUSB backend
  (`src/winusb/` — present and intact on `development`) rather than the Media Foundation path.
- Windows must bind **WinUSB** to the device. The T265 binding survives in
  `src/win7/drivers/IntelRealSense_D400_series_win7.inf:65` (`USB\VID_8087&PID_0B37`).
  Whether Windows 11 binds this automatically via MS OS descriptors or needs a manual driver
  install is **unverified** — expect to spend time here.
- Booting is two-stage: the unbooted device appears as Movidius `03E7:2150`, receives the
  firmware over a bulk transfer, then re-enumerates as `8087:0B37`. Both USB IDs are already
  present in `config/99-realsense-libusb.rules:55-56` (relevant to Linux; on Windows the
  driver binding is the equivalent concern).

**Hardware status at time of writing:** T265 not yet attached — a USB scan found no
`03E7:2150` or `8087:0B37` device. (Note that `VID_8087` alone is a false positive: Intel
wireless Bluetooth shares that vendor ID.)

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

### Phase 1 — Mechanical restore — ✅ **DONE 2026-07-18**

All 4,201 lines of `src/tm2/*` restored byte-identical from `391d5356e`, **deliberately not
wired into the build**. `realsense2` still builds clean (exit 0) and no `tm2` translation
unit is compiled. Baseline intact.

#### Method changed from the original plan — read this before Phase 2

The plan called for reverting the four removal commits and resolving the 10 conflicts. The
revert was run and then **aborted**, in favour of `git checkout 391d5356e -- src/tm2/`.

Reason: the 10 conflicts were never the risk. The revert *auto-merged* 23 other files without
complaint, and three of those changes were wrong:

| File | What the revert would have done | Why it's wrong |
|---|---|---|
| `third-party/rsutils/include/rsutils/version.h` | Revert `return( number != 0 )` → `return number` | Unrelated upstream cleanup, undone as collateral. Nothing to do with T265 |
| `common/rendering.h` | Rename `pose_to_world_transformation` → `tm2_pose_to_world_transformation` | Upstream deliberately generalised this name; the general name is live and in use |
| `CMake/lrs_options.cmake` | Set `BUILD_WITH_TM2` **ON by default** | Pulls unported `tm-device.cpp` into every build, destroying the baseline |

**Lesson for later phases: conflicts announce themselves, silent auto-merges do not.** Any
future use of `git revert` against these removal commits must audit the auto-merged set, not
just the conflicts.

#### Deferred hunks — where the rest of the revert went

Nothing was lost; each piece is deferred to the phase that owns it. Recover any of them with
`git show 391d5356e:<path>`, or re-run the revert and inspect.

| Deferred to | Files |
|---|---|
| **Phase 2** (enumeration) | `src/context.cpp`, `src/context.h` (`unload_tracking_module`) |
| **Phase 5** (API surface) | `src/core/motion.h` (`tm2_extensions`, `tm2_sensor_interface`), `include/librealsense2/hpp/rs_device.hpp`, `src/rs.cpp`, `wrappers/python/pyrs_device.cpp` |
| **Phase 7** (viewer/examples) | `common/model-views.cpp`, `common/viewer.cpp`, `common/rendering.h` |
| **Phase 8** (build/CI) | `CMake/lrs_options.cmake`, `CMake/global_config.cmake` (`add_tm2`), `CMake/unix_config.cmake`, `CMake/android_config.cmake`, `src/CMakeLists.txt`, `.github/workflows/buildsCI.yaml` |
| **Deliberately dropped** | `common/fw/*` (upstream removed FW bundling — see Phase 4), `unit-tests/unit-tests-live.cpp` (upstream migrated to pytest), `third-party/rsutils/.../version.h` (collateral) |

> Note: not one line of T265 code has been through a compiler yet. Phase 3 will produce a
> large volume of errors on first build; §3a predicts most of them, and comparing actual
> against predicted is the fastest check on whether that analysis was sound.

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

### Phase 3 — Port `tm_device` — the bulk of the work

Port 2,170 lines of `tm-device.cpp` onto the modern `backend_device` + sensor base classes.
`t265-messages.h` (1,532 lines) is pure wire protocol with no SDK dependencies and ports
verbatim.

**This phase implements `pose_sensor_interface`** — the missing piece that currently makes
`rs2::pose_sensor` throw for every device.

#### Sizing (static analysis, 2026-07-18 — see §5a for the API delta)

| Bucket | Lines | Share |
|---|---:|---:|
| Compiles untouched | ~1,450 | 67% |
| Mechanical adjustment | ~230 | 11% |
| Real rework | ~200 | 9% |
| New code outside the 2,170 | ~250 | — |

**Revised estimate: ~2.5–3 weeks with hardware in hand** (was: "2 weeks or 2 months, unknown").
Roughly 5–7 days to first clean compile, 3–5 days to first working pose stream, 3–5 days for
frame-pool and profile-tagging shakeout.

Three things that would have made this a multi-month job did not happen:

1. **`src/usb/*` is byte-for-byte unchanged** (copyright headers aside). Every
   `bulk_transfer` / `control_transfer` / `create_request` / `submit_request` call compiles
   verbatim. This was the largest and most fragile block of T265 code.
2. **`sensor_base` is still a valid direct base for a non-UVC, self-fed sensor.** There is no
   forced migration to `synthetic_sensor`, which would have been a real redesign. `sensor_base`
   is what `software_sensor` derives from today, so the pattern is still live.
3. **`frame_additional_data`'s 10-arg positional constructor survives byte-identical**, and
   `pose_sensor_interface` / `wheel_odometry_interface` plus their whole `rs.cpp` C-API
   implementation are intact — so the pose/relocalization/odometry surface needs **zero API
   design work**.

> These line counts are a projection from static analysis, not a measurement. Treat them as
> an order-of-magnitude guide — the useful conclusion is "weeks, not months," not the specific
> percentages.

#### Where the risk actually lives

Not in line count. Two runtime behaviors that a successful compile will **not** catch:

- **Frame archive keying changed.** `frame_source` now keys archives by
  `(stream, index, extension)` (`src/source.h:20`) rather than by extension alone. T265 pushes
  two fisheye streams through a single sensor; the old code shared one
  `RS2_EXTENSION_VIDEO_FRAME` archive across both, the new code will create two. The
  `set_max_publish_list_size(256)` at `tm-device.cpp:366` was tuned for the shared pool and now
  means 256 *per archive*. Expect to retune; expect frame-drop symptoms if it is missed.
- **`format_conversion` / `formats_converter` is new device-level machinery that T265
  predates** (`src/device.h:34`). T265 emits Y8 and 6DOF natively so conversion should be a
  no-op, but `device::tag_profiles()` interacts with it and T265 returned an empty tag vector
  (`tm-device.h:36`). This is a plausible source of "device enumerates but the pipeline won't
  start."

Both are only diagnosable against real hardware.

### Phase 3a — API delta checklist

Every SDK API `tm-device.cpp`/`.h` depends on that changed between `391d5356e` and
`origin/development`. Work through this during Phase 3.

**Signature changed — mechanical:**

| API | Old | Today | Sites |
|---|---|---|---|
| `sensor_base` ctor | 3 args (name, device*, owner) | 2 args (`src/sensor.h:58`) | 1 |
| `video_stream_profile` ctor | took `platform::stream_profile` | **arg-less** (`src/stream.h:111`) | 3 (`tm-device.cpp:465,499,542`) |
| `frame_source::alloc_frame` | `(rs2_extension, size, data, bool)` | `(archive_id, size, data&&, bool)` (`src/source.h:35`) | 3 (`:1118,1230,1534`) — copy `src/software-sensor.cpp:260`; needs `std::move` |
| `frame_source::set_sensor` | `shared_ptr` | `weak_ptr const&` (`src/source.h:44`) | 1 — implicit conversion, compiles as-is |
| `playback_device` ctor | `(ctx, reader)` | `(device_info, reader)` (`src/media/playback/playback_device.h:25`) | 1 — build a `playback_device_info` first |
| `backend_device_group` | uvc/usb | 4 lists incl. mipi | assertion at `:1922` needs a mipi term |

**Renamed / moved:**

| Old | Today |
|---|---|
| `frame_callback_ptr` | `rs2_frame_callback_sptr` (`src/core/sensor-interface.h:44`) |
| `notifications_callback_ptr` | `rs2_notifications_callback_sptr` (`:48`) |
| `frame_holder` in `archive.h` | `src/core/frame-holder.h:15` |
| `video_frame` / `pose_frame` / `frame` | `src/core/video-frame.h`, `core/pose-frame.h`, `src/frame.h` |
| `notification` | `src/core/notification.h:14` (brace-init still binds) |
| `librealsense::stream_profile` | `src/core/stream-profile.h:16` (added defaulted field) |
| `platform::stream_profile` | `src/platform/stream-profile.h` |
| `lazy<T>` | `rsutils::lazy<T>` |

**Removed — needs replacement:**

| Removed | Replacement | Sites |
|---|---|---|
| `environment::get_time_service()` | `librealsense::time_service::get_time()` (`src/core/time-service.h:19`, static) | 4 (`:1255,1455,1461,1564`) |
| `internal_frame_callback<T>` | `librealsense::make_frame_callback()` (`src/core/frame-callback.h:34`) | 1 (`:841`) |
| `hexify()` | `rsutils::string::hexdump` or a 3-line local helper | 1 (`:1981`) |
| `tm2_extensions` | **re-add** ~10 lines to `src/core/motion.h`; `RS2_EXTENSION_TM2` enum survives | — |
| `tm2_sensor_interface` | **re-add** ~10 lines; `RS2_EXTENSION_TM2_SENSOR` survives | — |
| `rs2_loopback_*` | un-stub in `src/rs.cpp:3424-3441` (~20 lines) | — |

**New requirements to satisfy:**

- `get_raw_stream_profiles()` is a **new pure virtual** on `sensor_interface`
  (`src/core/sensor-interface.h:38`) that `sensor_base` does not implement. Add:
  `stream_profiles const & get_raw_stream_profiles() const override { return initialized_profiles(); }`
- `pose_sensor_interface` and `wheel_odometry_interface` are **no longer `recordable<>`** —
  **delete** the four `create_snapshot`/`enable_recording` overrides at `tm-device.h:136-139`.
- Un-stub `src/rs.cpp:1960` and `:2001` (`return false` → `VALIDATE_INTERFACE_NO_THROW`).

**Confirmed unchanged — no work needed:** the whole `src/usb/*` layer; all 8 option classes
and `md_tm2_parser` (`tm-device.cpp:152-355`); `option_base` / `readonly_option` /
`register_option`; `environment::get_extrinsics_graph().register_extrinsics()`;
`register_stream_to_extrinsic_group`; `register_info` / `update_info`;
`frame_additional_data` 10-arg ctor; `frame_source::invoke_callback`; `dispatcher`;
`raise_on_before_streaming_changes`; `set_active_streams`; `ros_reader`; and the entire
`rs2_export/import_localization_map`, static-node and wheel-odometry C API
(`src/rs.cpp:3549-3648`).

### Phase 4 — Firmware delivery — 🟡 **INTERIM MECHANISM WORKING 2026-07-18**

Upstream deliberately deleted the bundling machinery, so `common/fw/` is not resurrected.

**Implemented:** `tm_boot()` loads the firmware image from the path in the
`RS2_T265_FW_PATH` environment variable and sends it over a single bulk transfer.
`src/tm2/tm-boot.cpp` is called from `rs_backend::query_usb_devices()`, so any unbooted
device is booted as a side effect of normal enumeration.

**This has been exercised against real hardware and works** — see the boot log below.

Deliberate departures from the original implementation:

- **Firmware from file, not a linked resource.** The original called `fw_get_target()` from
  the generated `common/fw/target.h`. That machinery is gone and is not coming back.
- **`.h`/`.cpp` split.** The original defined `tm_boot()` in the header, which only worked
  because exactly one translation unit included it.
- **Timeout raised 1s → 15s.** ~9 MB in a single bulk transfer is tight at one second even on
  a healthy link; the generous value keeps a slow-but-working transfer from being misreported
  as a failure.
- **Partial transfers reported distinctly from failures.** A short write means the link or
  timeout is at fault, not the image or endpoint — worth being able to tell apart at 3am.

**Still outstanding — the env var is not a shipping design.** It is fine for development but
requires every user to obtain and place a blob manually. A real answer (CMake fetch-and-verify
into a known location, a documented install path, or an internal mirror) is still owed, along
with the permanent mirror noted in §3.

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
| ~~No build toolchain installed~~ | ~~BLOCKER~~ → **RESOLVED** | §4a — toolchain installed and baseline `realsense2.dll` builds clean |
| ATL missing → MF backend unbuildable | Medium | §4a — blocked on a pending reboot. Does not block T265 (RSUSB path), but blocks any D400 non-regression claim on the default Windows backend |
| Frame-pool retuning (Phase 3) | **High** | Archive keying changed; `set_max_publish_list_size` now per-archive. Compile-clean but drops frames. Hardware-only diagnosis |
| Profile tagging / `format_conversion` | **High** | New machinery T265 predates; likely "enumerates but won't stream". Hardware-only diagnosis |
| No hardware CI | **High** | Upstream will never test this path; silent breakage on merge |
| ~~WinUSB driver binding on Win11~~ | ~~Medium~~ → **RESOLVED** | Windows 11 binds both the Movidius bootloader and the booted T265 with no manual driver install. A 9 MB bulk transfer to the unbooted device succeeded, so the binding is not merely present but usable |
| ~~Phase 3 effort unbounded~~ | ~~High~~ → **Medium** | **Downgraded** — sized at ~2.5–3 weeks; 67% of lines compile untouched |
| Firmware durability | Medium | Blob verified but not yet permanently mirrored |
| `libusb` / `WITH_TRACKING` regressions | Low–Medium | `libusb_config.cmake` and `external_libusb.cmake` both survive; `add_tm2()` should port verbatim |
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

| Level | Meaning | Status |
|---|---|---|
| Compiles | Builds with `BUILD_WITH_TM2=ON` | ✅ 2026-07-18 |
| Boots | Firmware pushed, device re-enumerates as `8087:0b37` | ✅ 2026-07-18 |
| Enumerates | Device is found and listed by librealsense | ✅ 2026-07-18 |
| Streams | Pose and fisheye frames arrive with sane values | ⬜ blocked on Phase 3 |
| Round-trips | Record and playback reproduce pose data | ⬜ |

**Evidence for the three achieved rungs** (hardware, 2026-07-18):

```
before:  USB\VID_03E7&PID_2150\03E72150            Movidius MA2X5X
         (unbooted T265 -- a Movidius bootloader)

$ RS2_T265_FW_PATH=target-0.2.0.951.mvcmd rs-enumerate-devices
  ERROR [rs2_create_device] Not Implemented
  T265 support is still being ported: the device enumerates but cannot yet be opened

after:   USB\VID_8087&PID_0B37\845412110485        Intel(R) RealSense(TM) Tracking Camera T265
         Status: OK
```

That error is the Phase 2 `create_device()` placeholder, and reaching it proves the whole
chain: firmware push → re-enumeration → `query_usb_devices` → factory hook →
`tm2_info::pick_tm2_devices` → device list → `create_device()`. The device is genuinely
recognised as a T265, serial 845412110485.

Phases 1–7 can reach *Compiles* with no hardware. **Nothing above that is provable without a
physical T265.** Until a device streams, this restoration is unverified regardless of how
much code has been committed — that gap should be stated in commit messages and status
updates, not glossed.

Do not mark a phase complete on the strength of a clean build alone.

## 8. Open questions

**Answered 2026-07-18:**

- ~~Is physical T265 hardware available for verification?~~ **Yes** — hardware is on hand and
  will be attached to the Windows 11 dev laptop. The full verification ladder is reachable.
- ~~Which platforms must be supported?~~ **Windows 11** is the dev/test target. Note this is
  the *harder* platform for T265: the WinUSB driver-binding question (§4a) does not arise on
  Linux, where udev rules already cover both USB IDs.

**Still open:**

- Where should the firmware blob be permanently mirrored? It must not go in git (9.3 MB
  binary), so it needs an internal artifact store or a documented fetch-and-verify step.
- Are ros2 bags required, or is the legacy rosbag path sufficient? (Determines whether the
  Phase 6 `ros2_writer.cpp` fix is in scope.)
- Where does this work get pushed? No `fork` remote is configured and `origin` is upstream
  (§Phase 0). Nothing can be pushed until this is decided.
- Is Linux support also wanted eventually? If so, much of the driver work is shared, but the
  platform-specific USB layer would need separate validation.
