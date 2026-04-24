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

| Timing                                          | Result                                                  |
| ----------------------------------------------- | ------------------------------------------------------- |
| Backend enabled + `count() > 0` before DAL init | Works correctly                                         |
| Backend enabled but `count() == 0` at DAL init  | DAL allocated, EKF sees 0 beacons forever               |
| Backend not `enabled()` at DAL init             | `_beacon` stays `nullptr`, EKF has no beacon DAL at all |

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

| Condition                               | Where                         |
| --------------------------------------- | ----------------------------- |
| `EK3_SRC1_POSXY = 4`                    | param                         |
| DAL beacon count > 0 at first EKF frame | `AP_DAL_Beacon` ctor snapshot |
| Beacon origin received by EKF           | `AP_NavEKF3_Measurements`     |
| Tilt aligned                            | `tiltAlignComplete`           |
| Yaw aligned (compass/GPS)               | `yawAlignComplete`            |
| Gyro bias converged                     | `delAngBiasLearned`           |
| 100 meas accumulated for centroid init  | `alignmentStarted`            |
| 100 more meas for filter convergence    | `alignmentCompleted`          |
| Fresh measurement present               | `rngBcn.dataToFuse`           |

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

| Driver     | NED frame origin                                  |
| ---------- | ------------------------------------------------- |
| Pozyx      | Pozyx system's map origin                         |
| Marvelmind | Hedgehog map origin (ENU → NED converted)         |
| Nooploop   | NodeFrame anchor map origin (ENU → NED converted) |

### What the EKF does with the origin

The EKF calls `get_origin()` (via DAL) solely to **seed its own EKF origin** when
transitioning to `AID_ABSOLUTE` via beacons. After that it works entirely in the beacon
NED frame.

### Practical rule

Set `BCN_LATITUDE/LONGITUDE/ALT` to any stable WGS-84 point (e.g. SITL home or a
real-world ground anchor). All beacon and vehicle positions passed via
`set_beacon_position()` / `set_vehicle_position()` must be NED metres relative to that
same point. The EKF will align its origin to it and navigate in that frame.

---

## Source-switch teleportation: GPS-origin vs beacon-origin mismatch

**Files:** `AP_NavEKF3_PosVelFusion.cpp:148`, `AP_NavEKF3_Control.cpp:462`,
`AP_NavEKF3_RngBcnFusion.cpp:71`

### Symptom

On switching EKF source to Beacons (source 3), the vehicle position teleports by a
large vector, snaps back after a few seconds, then teleports by ~2× the original vector,
then ~3×, etc.

### Observed values (pipe dashboard)

```
ekf.aid.pos_n/e  =  19.06,  112.97   ← vehicle in GPS EKF frame at switch moment
ekf.aid.rcv_n/e  = 301.42,  665.11   ← vehicle in beacon frame (3-state filter)
ekf.rst.delta_n/e= 282.36,  552.14   ← actual jump applied to EKF state (~618 m)
ekf.bof.ofs_n/e  =   0.00,    0.00   ← posOffsetNED is zero (correct post-reset)
```

### Root cause

`ResetPosition(RNGBCN)` in `PosVelFusion.cpp:150` unconditionally writes
`rngBcn.receiverPos` into `stateStruct.position`. That value is produced by the
3-state beacon pre-filter and is expressed **in the beacon NED frame** (origin =
`BCN_LATITUDE/LONGITUDE/ALT`). The main EKF state is expressed **in the GPS NED
frame** (origin = first GPS fix). When these two origins differ, the reset injects
a jump equal to the vector between them.

The reconciling `setOriginLLH()` call in `readRngBcnData()` is gated on
`!validOrigin` — but GPS has already set `validOrigin = true` before the source
switch, so the beacon origin is **never applied** to the EKF world frame.

### Why it doubles / triples

After the jump the EKF drifts back toward GPS (GPS fusion still active or timeout
recovery). When `originEstInit` is cleared and the cycle repeats, the beacon
3-state filter has re-converged to the same beacon-frame position while the EKF
state has returned to GPS-frame — so the next `ResetPosition` jump is the same
magnitude or larger depending on filter drift.

### Fix options

1. **Align the origins (parameter fix):** Set `BCN_LATITUDE/LONGITUDE/ALT` to the
   exact WGS-84 coordinate corresponding to the vehicle's position when the EKF
   first set its GPS origin (typically the arming/takeoff point). Then both frames
   share the same origin and the reset delta is ~0.

2. **Re-origin the EKF on source switch (code fix):** In `SelectAidingMode()` at
   the `readyToUseRangeBeacon()` branch, call `setOriginLLH(beacon->get_origin())`
   regardless of `validOrigin`. This re-anchors the EKF world frame to the beacon
   origin before the position reset, so the jump cancels out.

### Why vendor hardware never hits this bug

All three real beacon drivers (Pozyx, Marvelmind, Nooploop) operate in environments
where **GPS is absent or not the primary source at startup**. Their typical boot sequence:

1. Vehicle powers on indoors / in GPS-denied area.
2. Beacons accumulate 200 measurements → `alignmentCompleted`.
3. `readRngBcnData()` calls `setOriginLLH(beacon_origin)` because `!validOrigin` is
   still true — GPS has not fixed yet.
4. EKF origin is now anchored to `BCN_LAT/LON/ALT`.
5. `ResetPosition(RNGBCN)` fires: `stateStruct.position = receiverPos` — **both are
   already in the same frame**, so the jump is ~0.
6. `posOffsetNED` initialises to ~0 — correct, no translation needed.

None of the three drivers (Pozyx, Marvelmind, Nooploop) ever read or use
`BCN_LATITUDE/LONGITUDE/ALT` in code. They pass all positions in their hardware NED
frame and rely entirely on the user setting those parameters to the matching
real-world WGS-84 point so that `setOriginLLH` seeds the EKF with the right origin
during step 3 above.

**The `posOffsetNED` mechanism is designed for a second purpose:** when GPS is the
primary source (`AID_ABSOLUTE` with GPS) and beacons run in the background via
`FuseRngBcnStatic()`, a future switch to beacons should be seamless because
`posOffsetNED` would have captured any residual frame delta. In practice, for
vendor hardware starting without GPS, this offset is always ~0, so the mechanism
is never exercised.

### The scenario that breaks (our case)

1. SITL starts with GPS → GPS fixes → `validOrigin = true` → `EKF_origin` = takeoff point.
2. `setOriginLLH` gate (`!validOrigin`) is now permanently closed.
3. Beacon 3-state filter runs in background, converges in beacon frame
   (`BCN_LAT/LON/ALT` origin, **different** from takeoff point).
4. User switches to source 3.
5. `ResetPosition(RNGBCN)` writes beacon-frame `receiverPos` into GPS-frame state → **jump**.
6. `posOffsetNED = receiverPos − stateStruct.position = 0` (frames already collapsed by reset).
7. Beacon-position correction (line 1035) applies zero → no translation ever happens.

The core is not broken. It just has an untested code path:
**GPS-first followed by a live switch to beacons** — no vendor hardware does this.

### Verified with

`AP_PipeDash` logging added to:
- `AP_NavEKF3_PosVelFusion.cpp:148` — logs `ekf.rst.*` (from/to/delta)
- `AP_NavEKF3_Control.cpp:462` — logs `ekf.aid.*` (state pos + receiverPos at switch)
- `AP_NavEKF3_RngBcnFusion.cpp:71` — logs `ekf.bof.*` (posOffsetNED on first fusion tick)

---

# Origin alignment

Happens only once!
/home/ayakuba/src/poc/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp : 78
```c++
 if (!rngBcn.originEstInit) {
          rngBcn.originEstInit = true;
          rngBcn.posOffsetNED.x = rngBcn.receiverPos.x - stateStruct.position.x;
          rngBcn.posOffsetNED.y = rngBcn.receiverPos.y - stateStruct.position.y;
```

But `stateStruct.position is reset every second to 0/0 -> moving local origin:
/home/ayakuba/src/poc/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp : 2726
```c++
void NavEKF3_core::moveEKFOrigin(void) {

}
```

### Investigation result: `posOffsetNED` goes stale after `moveEKFOrigin`

**Files:** `AP_NavEKF3_core.cpp:2726`, `AP_NavEKF3_RngBcnFusion.cpp:78`,
`AP_NavEKF3_Measurements.cpp:1161`

#### How `posOffsetNED` is used

`readRngBcnData()` bakes the offset into every beacon measurement before fusion:

```cpp
// Measurements.cpp:1161
beacon_posNED.x += posOffsetNED.x;   // BCN frame → effective EKF frame
beacon_posNED.y += posOffsetNED.y;
```

`FuseRngBcn()` then computes:

```cpp
rngPred = |stateStruct.position - beacon_posNED_effective|
```

**Range-correctness invariant** — the predicted range equals the physical range iff:

```
|stateStruct.position − (beacon_BCN + posOffsetNED)| = |vehicle_BCN − beacon_BCN|
```

After `ResetPosition(RNGBCN)`, `stateStruct.position = receiverPos = vehicle_BCN`,
so `posOffsetNED = 0` satisfies the invariant. ✓

#### What `moveEKFOrigin` does to the invariant

```cpp
// core.cpp:2732-2745
loc = EKF_origin.offset(position.x, position.y);   // new physical origin
diffNE = loc.get_distance_NE_ftype(EKF_origin);     // ≈ −position
EKF_origin = loc;
stateStruct.position.xy() += diffNE;                // position → ~0
// posOffsetNED NOT touched  ← bug
```

After this: `stateStruct.position ≈ 0`, `posOffsetNED` still 0. Substituting:

```
|0 − (beacon_BCN + 0)| = |beacon_BCN|   ← distance from BCN origin to beacon
```

Correct range = `|vehicle_BCN − beacon_BCN|`. With vehicle at `(301, 665)` in BCN
frame and a beacon at `(400, 700)`, the error is:

```
wrong:   |(400, 700)| ≈ 806 m
correct: |(301−400, 665−700)| = |(−99, −35)| ≈ 105 m  →  ~700 m error
```

#### When `moveEKFOrigin` fires during beacon mode

Gate at `core.cpp:2728`:

```cpp
if (!frontend->common_origin_valid || !filterStatus.flags.using_gps) return;
```

`using_gps` flag (`Control.cpp:917`):

```cpp
status.flags.using_gps =
    ((imuSampleTime_ms - lastGpsPosPassTime_ms) < 4000) &&
    (PV_AidingMode == AID_ABSOLUTE);
```

Beacons also use `AID_ABSOLUTE`. So for **up to 4 seconds after the GPS→beacon
switch**, `using_gps` is true and `moveEKFOrigin` fires every second.

The update-loop ordering in `core.cpp:683-719` makes it land in the same frame:

```
SelectVelPosFusion()      ← GPS fusion, sets lastGpsPosPassTime_ms
SelectRngBcnFusion()      ← originEstInit fires, captures posOffsetNED
updateFilterStatus()      ← using_gps = true (GPS was recent)
moveEKFOrigin()           ← invalidates posOffsetNED in the same frame
```

#### Fix

The invariant is preserved when `posOffsetNED` receives the same delta as
`stateStruct.position`. Algebra: if `position → position + diffNE` must keep
`|position − (beacon + offset)|` constant, then `offset → offset + diffNE`.

Add to `moveEKFOrigin()` after the position/output-state block:

```cpp
// core.cpp: after line 2745 (storedOutput loop)
#if EK3_FEATURE_BEACON_FUSION
  // Keep beacon frame offset consistent with the origin shift.
  // When EKF origin moves by diffNE, position drops by diffNE,
  // so posOffsetNED must compensate by the same amount.
  if (rngBcn.originEstInit) {
    rngBcn.posOffsetNED.xy() += diffNE;
  }
#endif
```

Verification with example above:
- `diffNE ≈ (−301, −665)`, `posOffsetNED_old = 0`
- `posOffsetNED_new = (−301, −665)`
- `beacon_effective = (400, 700) + (−301, −665) = (99, 35)`
- `rngPred = |(0,0) − (99, 35)| = 105 m` ✓

#### Why vendor hardware never hit this

All three real drivers (Pozyx, Marvelmind, Nooploop) start without GPS.
`using_gps` is always false → `moveEKFOrigin` never fires → `posOffsetNED`
stays at its initial value of 0. The bug is exclusive to the GPS-first-then-switch path.

---

## Tuning: reducing EKF trust in beacon measurements (rely more on INS)

**Files:** `libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp:114`,
`libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp:1079`

### How beacon influence is weighted

The Kalman gain that controls how much each beacon measurement corrects the EKF state is:

```
K = P / (P + R_BCN)
```

`R_BCN` is computed at fusion time as:

```cpp
// RngBcnFusion.cpp:114
const ftype R_BCN = sq(MAX(rngBcn.dataDelayed.rngErr, 0.1f));
```

`rngErr` is set directly from the `EK3_BCN_M_NSE` parameter:

```cpp
// Measurements.cpp:1079
rngBcnDataNew.rngErr = frontend->_rngBcnNoise.get();
```

Larger `R_BCN` → smaller Kalman gain → each measurement corrects the state less → EKF
relies more on INS dead-reckoning between measurements.

### Parameters

| Parameter       | Default | Effect                                                                    |
| --------------- | ------- | ------------------------------------------------------------------------- |
| `EK3_BCN_M_NSE` | `1.0` m | **Primary lever.** Increase to reduce beacon trust. `R_BCN = sq(value)`, so doubling it quarters the effective influence. |
| `EK3_BCN_I_GTE` | `500` % | Innovation gate width. Decrease to outright reject measurements that deviate too far from predicted range (hard cut, not soft downweighting). |

### Recommended starting points

```
EK3_BCN_M_NSE = 3.0   # ~9× less beacon influence than default; try 2.0–10.0
EK3_BCN_I_GTE = 200   # tighter gate to discard noisy outliers
```

To trust INS predictions more independently, also lower the IMU process noise:

```
EK3_ACCEL_P_NSE   # decrease → EKF is more confident in its IMU prediction → lower K
EK3_GYRO_P_NSE    # same effect
```