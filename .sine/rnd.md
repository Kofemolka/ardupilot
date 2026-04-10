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
