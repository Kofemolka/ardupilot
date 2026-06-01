# Ranging System Dead-Lock Analysis & Re-engagement Fix

## Problem

Once the beacon ranging system becomes unhealthy, it can lock out permanently.

Both `FuseRngBcn()` and `FuseRngBcnStatic()` gate all corrections behind a
consistency check:

```
testRatio = innov² / (gate² × varInnov)
health    = (testRatio < 1.0f) || badIMUdata
```

When `health == false`, no correction is applied. The position estimate
(`stateStruct.position` in primary mode, `receiverPos` in static mode) drifts
further from truth, growing the innovation, making `testRatio` worse. This is a
positive-feedback loop with no exit.

### Why process noise alone is not enough

`FuseRngBcnStatic()` adds `0.1 m²/step` to `receiverPosCov[i][i]`
unconditionally, which inflates `varInnov` and can theoretically bring
`testRatio` back below 1. In practice this fails when:

- The position error is large enough that even with `varInnov` fully inflated
  after several seconds, `innov²` still dominates.
- `FuseRngBcn()` (primary mode) has no equivalent — it relies on the full EKF
  covariance, which is held small by any other active position source (GPS,
  ExtNav).

### What makes `alignmentCompleted` a one-way door

`alignmentCompleted` is set `true` after 100 fused measurements and is never
cleared. Once set, the health-bypass clause `|| !alignmentCompleted` in
`FuseRngBcnStatic()` is permanently gone, and `FuseRngBcn()` is the only route
in primary beacon mode. There is no watchdog, timeout, or fallback.

### `posOffsetNED` is a frozen frame transform

`originEstInit` is set once at the first entry into primary beacon mode
(line 72). The XY offset between beacon-frame and EKF-frame is computed there
and never revisited. Long flights with GPS drift cause this offset to go stale,
adding a systematic bias to every innovation computed by `FuseRngBcn()`.


## Dead-System Criteria

Three conditions must all be true simultaneously:

| Condition | Meaning |
|---|---|
| `alignmentCompleted == true` | Was working before (not just initialising) |
| `dataToFuse == true` | Measurements are arriving but being rejected |
| `(imuSampleTime_ms - lastPassTime_ms) > 5000 ms` | No healthy fusion for 5 s |

The 5-second threshold is chosen because at 25 Hz the process noise adds
`0.1 × 25 × 5 = 12.5 m²` per axis to `receiverPosCov`. For `testRatio` to
still exceed 1 at that point, the position error must be `> gate × √12.5 ≈ 17 m`.
An error that large will not self-correct via process noise.

The existing 1-second "BAD" GCS display (line 106) is a *degraded* signal.
5 seconds is the *dead* signal.


## Solution: Jump to Phase 2 on Dead Detection

### Why phase 1 can be skipped

Phase 1 (`!alignmentStarted`) accumulates 100 beacon observations and seeds
`receiverPos` with the beacon constellation centroid at `z = 0`. This seed is
wrong for a flying vehicle. When a dead-reset is triggered, a better seed is
already available: `stateStruct.position` (current EKF estimate).

Phase 2 (`alignmentStarted && !alignmentCompleted`) fuses all 100 measurements
unconditionally via the `|| !alignmentCompleted` health bypass, regardless of
innovation size. Starting from a good seed (`stateStruct.position`), convergence
is fast. Starting from a bad seed it still converges — the bypass ensures it.

### What the reset does

```cpp
// AP_NavEKF3_RngBcnFusion.cpp — SelectRngBcnFusion(), top of dataToFuse block
if (rngBcn.alignmentCompleted &&
    (imuSampleTime_ms - rngBcn.lastPassTime_ms) > 5000U) {
  rngBcn.alignmentCompleted = false;       // re-enable health bypass
  rngBcn.numMeas = 0;                      // re-arm 100-step countdown
  rngBcn.receiverPos = stateStruct.position; // seed from EKF, not beacon centroid
  memset(rngBcn.receiverPosCov, 0, sizeof(rngBcn.receiverPosCov));
  rngBcn.receiverPosCov[0][0] =
  rngBcn.receiverPosCov[1][1] =
  rngBcn.receiverPosCov[2][2] = 100.0f;   // 10 m stddev → high Kalman gain
  PIPE("rng.dead.reset", 1);
}
```

`alignmentStarted` is deliberately not touched — it stays `true` so phase 1
data collection is skipped entirely.

### Cascade through primary mode

When `alignmentCompleted` is cleared:

1. The routing condition at line 70 (`&& rngBcn.alignmentCompleted`) fails, so
   `FuseRngBcn()` is not called even if the source is BEACON.
2. `FuseRngBcnStatic()` runs instead, and lines 87/94 reset `originEstInit = false`.
3. After 100 healthy static-filter measurements, `alignmentCompleted` becomes
   `true` again.
4. Primary mode re-engages on the next step. Because `originEstInit == false`,
   `posOffsetNED` is recomputed from the freshly converged `receiverPos` — the
   stale frame transform is corrected automatically.

The fix therefore works for both static and primary fusion modes through the
same single trigger.

### Re-trigger safety

On the step the dead-check fires, health is immediately forced `true` by the
`|| !alignmentCompleted` bypass, so `lastPassTime_ms` is updated on the very
next fused measurement. The 5-second window resets. A re-trigger only occurs if
convergence genuinely fails (e.g., `stateStruct.position` is also bad), which
produces a periodic retry every 5 seconds — an acceptable fallback.


## Limitations

- If `stateStruct.position` is also drifted (GPS lost simultaneously), the seed
  is imprecise. Phase 2's health bypass still forces convergence, but it takes
  more of the 100 steps to pull `receiverPos` to truth.
- The 5-second threshold is fixed. A vehicle flying fast during the dead period
  may have moved far enough that even `stateStruct.position` is a coarse seed.
  Increasing `receiverPosCov[i][i]` beyond 100 m² would accommodate that at the
  cost of slower convergence.
- There is no equivalent re-engagement mechanism for the `posOffsetNED.z`
  vertical offset (`posDownOffsetMax/Min` dual-hypothesis filter). Vertical
  recovery relies on the existing `CalcRangeBeaconPosDownOffset` logic resuming
  once horizontal fusion is healthy.


---

## Attempt: phase-2 jump — observed failure

### What the flight data showed

Log `00000150.BIN`. Aircraft flew SW→NE, ranges growing from ~5000 m to ~11000 m
as it moved away from the beacon cluster. At ~t=1100 s the innovation began
diverging to −300…−500 m (predicted range << measured range), confirming
`receiverPos` stopped tracking the aircraft and remained near the beacon
centroid. EKF source set (`XKRP`) showed repeated SRC2/SRC3 fallbacks throughout.

### Why the seed-from-EKF approach fails when EKF position is large-error

The dead-check resets `receiverPos = stateStruct.position`. This is only useful
when `stateStruct.position` is close to truth. When EKF position has also
drifted far (e.g., because beacons were the only position source and fusion was
dead for minutes), both the seed and the true position are far apart.

The static filter's Kalman update is **linearised** around `receiverPos`:

```
H_RNG = unit vector from receiverPos to beacon
innov = |receiverPos - beacon| - range_measured
correction = K × innov  (K ≈ 1 when cov >> R)
```

When `receiverPos` is thousands of metres from truth, the Jacobian direction is
wrong. A gain-1 correction moves `receiverPos` along the wrong gradient. The
filter does not converge — it bounces between beacon range shells without
finding the intersection.

Even if `receiverPos` somehow converged after phase 2, `stateStruct.position`
is **never touched** by the static filter. When `alignmentCompleted` flips and
`FuseRngBcn()` resumes, its innovation is:

```
innov = |stateStruct.position - beacon| - range_measured
```

`stateStruct.position` is still the drifted value → innovation is still
thousands of metres → health fails on the first step → system is dead again
immediately.

### Why the Control.cpp interaction matters

`lastPassTime_ms` is also used at `AP_NavEKF3_Control.cpp:398,418` for
`attAidLossCritical` and `posAidLossCritical`. Updating it from
`FuseRngBcnStatic()` would suppress safety-critical mode changes (e.g., the
filter reverting to `AID_NONE`) even when beacons are not correcting EKF state.
This regression was caught and the static-mode update was reverted. The
dead-check reset now stamps `lastPassTime_ms = imuSampleTime_ms` only in the
dead-check block itself (primary-beacon-mode guard applied), giving a recovery
window without permanently suppressing the loss timers.

### Why the convergence-count reduction was reverted

Initial attempt replaced `numMeas >= 100` with a covariance threshold `< 4.0 m²`
plus `3 × numFusionReports` floor. Both constants are arbitrary:

- `4.0 m²` (2 m stddev) has no relationship to `_rngBcnInnovGate`, `rngErr`, or
  any EK3 parameter. With low beacon rate, steady-state covariance may exceed
  this threshold indefinitely due to process noise between measurements.
- `3 × numFusionReports` bakes in round-robin cycling and rate assumptions
  specific to one beacon type, not applicable to all EK3 users.

`FuseRngBcnStatic()` is shared EK3 code. The change was confined to the
dead-check reset: `numMeas = 90` so only 10 measurements are needed after
re-seeding. The reasoning: 10 measurements is enough to verify the seed is
consistent with the incoming ranges, not to achieve cold-start convergence.
But this is irrelevant when the seed itself is wrong (see above).


---

## Next direction: MLAT-based position fix before EKF reset

### Principle

When the dead-check fires and `stateStruct.position` cannot be trusted, the
only position information available is the set of raw range measurements. With
3+ beacons, multilateration (MLAT) gives a closed-form or iteratively refined
position fix that is **independent of `stateStruct.position`**. This fix does
not use a Kalman linearisation, so it is not affected by how far wrong the
current estimate is.

The proposed sequence:

1. Dead-check fires (beacon dead for 5 s, source = BEACON).
2. Collect the most recent range from each available beacon (from the data
   buffer, not new measurements — latency is acceptable here).
3. Run MLAT several times (e.g., 3–5 iterations of a least-squares solver or
   Taylor-series linearisation seeded from beacon centroid) to obtain a
   position fix `mlat_pos`.
4. Validate the fix: residuals of all ranges against `mlat_pos` must be below a
   threshold. If validation fails (poor geometry, fewer than 3 beacons), abort
   and retry at next dead-check interval.
5. Seed `receiverPos = mlat_pos` and `stateStruct.position = mlat_pos` (soft
   EKF position reset: inflate `P[7][7]`, `P[8][8]`, `P[9][9]` to reflect the
   MLAT accuracy, reset velocity if needed).
6. Re-enter phase 2 (`numMeas = 90`, `alignmentCompleted = false`,
   `receiverPosCov[i][i]` = MLAT residual variance).
7. Static filter now converges from a valid linearisation point. After 10
   measurements, `alignmentCompleted` flips, primary mode resumes with a correct
   `posOffsetNED`.

### Why soft EKF reset is necessary

Without resetting `stateStruct.position`, `FuseRngBcn()` will always recompute
a large innovation on re-entry (step 7 above fails). The `posOffsetNED.x/y`
computed at re-engagement does not help — it is not applied in the `FuseRngBcn()`
innovation (only `posOffsetNED.z` is applied to the beacon z-coordinate). A
soft reset moves the EKF state to the MLAT fix and inflates the covariance so
the EKF can immediately absorb the first few range measurements.

### Open questions before implementation

- Where to implement MLAT: new function in `AP_NavEKF3_RngBcnFusion.cpp`, or
  delegated to `AP_Beacon`?
- Minimum beacon count for a reliable fix: 3 for 2D (if altitude known), 4 for
  full 3D. With 2 beacons, MLAT is underdetermined — fall back to a
  1D correction along the baseline only?
- How many MLAT iterations to run per dead-check firing? Trade-off: more
  iterations = better accuracy, more CPU. 5 iterations of Gauss-Newton should
  be sufficient for sub-metre convergence when seeded from beacon centroid.
- Soft reset scope: position only, or also velocity? If the aircraft was flying
  when beacons died, the velocity estimate from IMU integration may still be
  valid — resetting it would introduce error. Reset position only, keep velocity.
- Handling of `posOffsetNED` after reset: since both `receiverPos` and
  `stateStruct.position` are set to `mlat_pos`, `posOffsetNED.x/y` will be zero
  at re-engagement. This is correct — the two frames are now aligned.
