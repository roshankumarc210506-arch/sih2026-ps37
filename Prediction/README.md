# M2 — Prediction Module

Owner: M2
Status: Phase 0 and Phase 1 complete (see Known Limitations for what still depends on Day 3 data)

## What this module does

Consumes the perception module's track list and produces short-horizon predicted trajectories with per-class growing uncertainty cones, for the global planner to use as occupancy-map inflation.

## Interface contract

**Input** — `SihTrackBus`, a fixed-size 40-element array (`tracks`) plus `num_tracks` (uint32):

```
{id, class, x, y, heading, velocity, covariance, valid}
```

- `class` is the `AgentClass` enum (`Car`, `TwoWheeler`, `AutoRickshaw`, `PushCart`, `Pedestrian`, `Animal`, `Unknown`)
- `covariance` is 4×4 over `[x, y, heading, velocity]`, already Jacobian-transformed by M1 from the tracker's internal state — never re-derive from raw state
- Frame: ego-vehicle, ISO 8855 (x forward, y left, θ CCW)
- Loop `1:num_tracks` and check `.valid` — slots beyond that are padding, not real tracks

**Output** — `SihPredictionBus`, a fixed-size 40-element array:

```
{id, valid, predicted_positions [20x2], uncertainty_radius [20x1]}
```

- Horizon N = 20 steps
- Same `valid`/padding convention as the input
- `predicted_positions` is in the same ego-vehicle ISO 8855 frame as the input

Both bus sizes track `cfg.MaxTracks` (currently 40, changed from 20 on Day 2 by M1 — see `buses/sihDefineBuses.m` for the canonical value).

## Files

| File | Purpose |
|---|---|
| `predictMotionCVCTRBlock.m` | Core MATLAB Function block — CV/CTR prediction, per-class uncertainty growth, local yaw-rate estimation |
| `classUncertaintyParams.m` | Per-class `r0`/`alpha`/`growthExponent` — currently fit to Day 1 stub-sensor error, **must be retuned against Day 3 real-sensor data** |
| `classUncertaintyParams_day2_STUB.m` | Snapshot of the Day 2 stub-fit parameters, kept for provenance/comparison once real tuning lands |
| `createPredictionBusObjects.m` | Defines `SihPredictionBus` as a flat 40-element array (see Known Limitations for why it's flat, not nested) |
| `predictionModuleTest.slx` | Standalone test harness — `DummyTrackGenerator` source block replaying M1's exported perception data frame-by-frame |
| `m2_predictions_day2_STUB.mat` | Exported multi-frame prediction output for M3 to develop against — stub-data-derived, will be replaced |
| `AgentClass.m`, `sihCreateBuses.m`, `sihDefineBuses.m` | **Not owned here** — reference `buses/` directly, do not keep local copies (see Known Limitations, this bit us twice) |

## Design decisions

**Uncertainty model:** `r(t) = r0 + alpha * t^growthExponent`. `r0` comes from the largest eigenvalue of the covariance's 2×2 position block (symmetrized, forced real for codegen). `alpha`/`growthExponent` are per-class, currently fit to the 95th percentile of measured prediction error at each horizon step (not the mean — the planner's collision check is a hard binary boundary, so under-sizing the cone half the time is a real collision risk, not just a quality issue).

**Yaw rate:** M1's track bus does not carry yaw rate. Rather than request a contract change immediately, M2 estimates it locally via heading differencing between consecutive frames of the same track, gated on (a) matching track ID across frames and (b) a plausibility bound rejecting >1.5 rad/s jumps as tracker noise rather than real turning. This activates the CTR branch of the predictor for genuinely turning tracks. Confirmed working: `predicted_positions` show measurable curvature for tracks with consistent heading change across frames. If this estimate proves too noisy on Day 3 real sensor data, M2 will request `yawRate` be exposed properly from M1's tracker state — that would be a genuine contract change requiring team-wide notify.

**Warm-up tracks:** M1's tracker emits `heading = 0` for newly-initialized tracks during warm-up, indistinguishable from a track genuinely heading along +x. Until this is resolved upstream, `baseUncertaintyRadius` widens the cone (×2) for tracks where both heading and velocity read as zero, rather than trusting an unreliable direction.

**`Unknown` class:** A meaningful share of real tracks come back unclassified (roughly half in early testing, at M1's current ~64.5% classifier accuracy). These were previously falling through silently to `Pedestrian`'s uncertainty parameters, which are among the tightest — an unknown fast-moving object would get an undersized cone. Added an explicit `Unknown` entry with the most conservative growth rate in the table.

## Known limitations

**Bus shape reverted from M6's nested container design.** M6's locked contract specifies `predictions.agents[40]` / `.num_agents` / `.timestamp` — a struct containing a struct array. This shape cannot be constructed inside a MATLAB Function block: codegen rejects any indexed assignment into a struct-array field of a struct (`predictions.agents(i).field = ...`), regardless of construction order. Reverted to a flat 40-element bus array as a working interim. Flagged to M6 — either downstream consumers wrap this with a Bus Creator to get the container shape, or the container gets built in a Simulink subsystem rather than a MATLAB Function block. **M3 should confirm which shape they're actually wiring against before building on it.**

**Tuning is provisional.** `classUncertaintyParams.m` is fit against `m1_perception_day1.mat`, which is Day 1 stub-sensor data, not real camera/radar/LiDAR. The fit itself is sound (95th-percentile error per class, measured against M1's own logged ground truth across the full 248-frame sequence), but the *numbers* will shift once Day 3's consolidated real-sensor export lands. Some classes (`Unknown`, `Animal`) have thin sample support at long horizons (n=2–6) and were deliberately inflated beyond the raw fit as a safety margin — treat their exact values as more provisional than `Car`/`AutoRickshaw`/`TwoWheeler`/`PushCart`, which had strong sample sizes throughout.

**Finding worth flagging:** on this dataset, `Car` needed the *widest* uncertainty cone of any class (fastest-growing, by a wide margin) — not the tightest, which was the initial placeholder assumption. If this holds on real data, it's a real correction to how the planner should weight vehicle vs. pedestrian/animal risk.

**Do not keep local copies of shared files.** Local copies of `AgentClass.m`/`sihCreateBuses.m`/`sihDefineBuses.m`/`sihConfig.m` inside this module's folder were found and deleted four separate times during Phase 1 — they were silently shadowing the real shared files in `buses/` and caused several hours of misdiagnosis. Always reference `buses/` directly.

**`sihDefineBuses.m` currently hard-asserts on all six members' bus scripts being present.** This will block anyone who doesn't have every teammate's script on their path yet. Flagged to M6 as worth downgrading to a warning.

## Validated

- CV propagation matches hand-calculated values to 4 decimal places
- Frame convention (ISO 8855) confirmed — heading in equals travel direction out
- Uniform step spacing (`v*dt`) across the horizon
- Radii monotonic, covariance-derived, class-dependent
- Full 248-frame sequence runs clean (`MAX_TRACKS=40`, `N=20`)
- Track appearance/disappearance: 444 track IDs catalogued across the run, zero ID-reuse gaps found, slots confirmed fully cleared (no stale data) the frame after a track disappears — verified across 34 unused slots in a low-occupancy frame
- CTR branch confirmed active — curved (non-straight) predicted paths observed for turning tracks
- Per-class uncertainty growth rates fit to measured p95 prediction error

## Not yet done (post-Phase 1)

- Retune `classUncertaintyParams.m` against M1's Day 3 consolidated real-sensor export
- Re-validate appearance/disappearance and yaw-rate noise characteristics on real (not stub) data
- Resolve the bus-shape question with M6 (flat array accepted permanently, or container built elsewhere)
- Decide whether the local yaw-rate estimator is sufficient long-term or whether to request `yawRate` as a proper contract addition from M1
