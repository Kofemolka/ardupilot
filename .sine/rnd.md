# Research Notes

## AP_DAL_Beacon: beacon count is frozen at construction

**Files:** `libraries/AP_DAL/AP_DAL_Beacon.cpp`, `libraries/AP_DAL/AP_DAL.cpp`

### Key finding

`AP_DAL_Beacon` is only allocated if `bcn->enabled()` is true at DAL init time
(`AP_DAL.cpp:155-159`). Inside the constructor (`AP_DAL_Beacon.cpp:13`), the beacon
count is snapshot once:

```cpp
_RBCH.count = bcon->count();
```

`start_frame()` refreshes position, origin, health, and distance every EKF frame —
but **never updates `count`**. The `count()` accessor reads only from `_RBCH.count`.

### Consequence

| Timing | Result |
|---|---|
| Backend enabled + `count() > 0` before DAL init | Works correctly |
| Backend enabled but `count() == 0` at DAL init | DAL allocated, EKF sees 0 beacons forever |
| Backend not `enabled()` at DAL init | `_beacon` stays `nullptr`, EKF has no beacon DAL at all |

### Rule

The beacon backend **must** report its final beacon count before `AP_DAL::init()` runs
(called once at EKF startup). Drivers that populate beacons lazily from MAVLink messages
will snapshot `count = 0` and the EKF will ignore all subsequent beacon data.

---

## EKF position check chain: what beacons must satisfy for `has_position`

**Files:** `ArduCopter/ekf_check.cpp`, `ArduCopter/system.cpp`, `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp`, `libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp`

### Call chain

```
ekf_check.cpp:53  has_position = ekf_has_relative_position() || ekf_has_absolute_position()
                                                                         ↓
                                          ahrs.has_status(HORIZ_POS_ABS)
                                                                         ↓
                               filterStatus.flags.horiz_pos_abs = doingNormalGpsNav && filterHealthy
                                                                         ↓
                               doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE)
                                                                         ↓
                               PV_AidingMode set to AID_ABSOLUTE only when readyToUseRangeBeacon()
```

Beacons **never** satisfy `horiz_pos_rel` — that requires optflow, visual odometry, or
dead reckoning. Beacons only contribute `horiz_pos_abs`.

### `readyToUseRangeBeacon()` — all 5 gates must be true (`Control.cpp:602`)

```cpp
return tiltAlignComplete        // IMU tilt variance < 5° (~few seconds after boot)
    && yawAlignComplete         // compass or GPS yaw locked
    && delAngBiasLearned        // gyro bias converged
    && rngBcn.alignmentCompleted  // beacon pre-filter bootstrapped (see below)
    && rngBcn.dataToFuse;       // fresh measurement in the queue right now
```

### `rngBcn.alignmentCompleted` — two-phase bootstrap (`RngBcnFusion.cpp:339-374`)

- **Phase 1** (`alignmentStarted`): accumulate 100 unique-beacon measurements → initialise
  a 3-state receiver position filter to the centroid of all beacon positions.
- **Phase 2** (`alignmentCompleted`): run that filter for 100 more iterations.

With 3 beacons at 134 ms cycle → ~7.5 meas/s/beacon → ~3 seconds total before this clears.

### Gate zero: parameter

`EK3_SRC1_POSXY` must be `4` (BEACON). Checked before everything else in `readyToUseRangeBeacon()`.

### Full checklist

| Condition | Where |
|---|---|
| `EK3_SRC1_POSXY = 4` | param |
| DAL beacon count > 0 at first EKF frame | `AP_DAL_Beacon` ctor snapshot |
| Beacon origin received by EKF | `AP_NavEKF3_Measurements` |
| Tilt aligned | `tiltAlignComplete` |
| Yaw aligned (compass/GPS) | `yawAlignComplete` |
| Gyro bias converged | `delAngBiasLearned` |
| 100 meas accumulated for centroid init | `alignmentStarted` |
| 100 more meas for filter convergence | `alignmentCompleted` |
| Fresh measurement present | `rngBcn.dataToFuse` |

---

## Beacon NED frame is independent from EKF origin

**Files:** `libraries/AP_Beacon/AP_Beacon.h`, `libraries/AP_Beacon/AP_Beacon.cpp`, `libraries/AP_Beacon/AP_Beacon_Backend.h`

### Key finding

The beacon NED origin is **not** the EKF origin. It is defined by three plain `AP_Float`
parameters on the `AP_Beacon` frontend:

```
BCN_LATITUDE   → origin_lat
BCN_LONGITUDE  → origin_lon
BCN_ALT        → origin_alt
```

The backend reads them via:
```cpp
get_beacon_origin_lat() / get_beacon_origin_lon() / get_beacon_origin_alt()
```

`AP_Beacon::get_origin()` just converts those floats to a `Location` struct — no EKF
involvement at all.

### How all vendor drivers use it

None of the real-hardware drivers (Pozyx, Marvelmind, Nooploop) set `origin_lat/lon/alt`
from code. They only call two backend helpers:

- `set_beacon_position(id, Vector3f NED)` — beacon position in their own NED frame
- `set_vehicle_position(Vector3f NED, accuracy)` — vehicle position in that same frame

Each driver uses its hardware system's internal map as the NED frame, and relies on the
user setting `BCN_LATITUDE/LONGITUDE/ALT` to the matching real-world WGS-84 coordinate.

| Driver | NED frame origin |
|---|---|
| Pozyx | Pozyx system's map origin |
| Marvelmind | Hedgehog map origin (ENU → NED converted) |
| Nooploop | NodeFrame anchor map origin (ENU → NED converted) |

### What the EKF does with the origin

The EKF calls `get_origin()` (via DAL) solely to **seed its own EKF origin** when
transitioning to `AID_ABSOLUTE` via beacons. After that it works entirely in the beacon
NED frame.

### Practical rule

Set `BCN_LATITUDE/LONGITUDE/ALT` to any stable WGS-84 point (e.g. SITL home or a
real-world ground anchor). All beacon and vehicle positions passed via
`set_beacon_position()` / `set_vehicle_position()` must be NED metres relative to that
same point. The EKF will align its origin to it and navigate in that frame.
