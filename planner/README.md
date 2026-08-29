# Global Planner (M3)

Owns `plannerHybridAStar` + the velocity profiler that turns its output into
the locked `Plan bus to MPC`.

## Calling convention (REQUIRED)

`assignVelocityProfile` intentionally throws `TooFewWaypoints` on a plan with
fewer than 2 waypoints - this is a deliberate design choice, not a bug to
work around. A failed `planGlobalPath` call (`IsPathFound=false`, whether
from `BadStart`/`BadGoal`/`SearchFailed`) always has 0 waypoints, so it
**must** be routed through `emptyVelPlan.m` instead:

```matlab
gp = planGlobalPath(omap, cfg, mapInfo, startPose, goalPose);

if gp.IsPathFound
    vp = assignVelocityProfile(gp, cfg);
else
    vp = emptyVelPlan(gp, cfg);   % NOT assignVelocityProfile - it will throw
end

bus = packPlanBus(vp, gp, cfg);
```

**Confirmed by test** (`testFailedPlanThroughBus.m`): following this
convention, a failed plan produces `bus.NumWaypoints == 0` cleanly, with no
error anywhere in the chain. This is a safe integration gate - but only if
callers use `emptyVelPlan.m` on failure. Calling `assignVelocityProfile`
directly on a failed plan will throw, not return a zero-waypoint result.

`planResult.FailureReason` is set to one of `'BadStart'`, `'BadGoal'`, or
`'SearchFailed'` for diagnostics, but is **not** part of the locked bus
schema - `NumWaypoints==0` is the only failure signal on the bus itself.

`packPlanBus` keeps a persistent `SeqNum` that increments across calls in
the same MATLAB session. Pass `resetSeq=true` to reset it (useful at the
start of a fresh test run, so `SeqNum` doesn't carry over unexpectedly).

## Real-data loaders

- `loadM1RealPerception.m` - reads one timestep of M1's real perception
  export, snapping to the nearest sample to a requested `timeQuery`.
- `loadM2RealPredictions.m` + `computeM1FirstSeenTimes.m` - M2's prediction
  `timeseries` objects use a **relative per-agent clock** (each starts at
  `Time=0` when that agent's own horizon starts), **not** global sim time.
  Always go through the offset map (`computeM1FirstSeenTimes`) to convert a
  global time query into each agent's relative clock - never index M2's
  `.mat` file directly by a shared time index.

Data file locations (not pushed by M3 - these are M1's/M2's own exports):
- `data/m1_perception_day1.mat`
- `Prediction/m2_predictions_day1.mat` - note the **capital-P** folder,
  inconsistent with the repo's `data/` convention elsewhere. Don't assume
  a `data/`-scoped glob will find it.

## Known limitations

- **Footprint**: single-disc mode, 2.52m radius, rear-axle referenced
  (locked per contract). Confirmed under-coverage at the nose: 1.385m
  (front bumper sits outside the disc). M4's NMPC independently covers this
  via a 3-disc constraint on the control side - not purely M3's exposure.
- **Static obstacles**: road edges only, via `roadBoundaries(scenario)`.
  No support yet for `barrier()`-based obstacles (market stalls, walls) -
  open question with M5, not yet answered.
- **`AgentClass.Unknown`**: present in real M1 data. Treated conservatively
  - mapped to `Animal`'s risk margin and `Car`'s diagonal extent (the
    largest values in each table) in `classToRiskField.m` /
    `classToAgentExtent.m`. Every observed `Unknown` track had
    `heading=0, velocity=0` - flagged to M1, not yet confirmed whether
    that's expected classifier behavior.
- **M1/M2 ID matching**: tested at one real timestep (9 real M1 tracks, 11
  real M2 predictions) - 6 of 9 tracks had no matching prediction. Code
  degrades gracefully (falls back to 0 uncertainty growth for unmatched
  tracks, doesn't crash) but this is a real gap, not yet resolved with M2.
- **`findOnRoadPoint.m`** hardcodes `heading=0` for any start/goal it finds.
  A tangent-based heading estimate was tried and reverted - it broke real
  pathfinding on the actual (narrow) village road. Diagnostic-helper-only,
  low priority per original scope - don't assume it returns a meaningful
  heading, only a valid on-road position.
- **`SihEgoBus.Timestamp`** (from M4) isn't wired into the real-data loaders
  yet - they work directly off the exported `.mat` time arrays. Matters for
  live-Simulink integration (M6), not today's offline `.mat`-based testing.

## Regression fixture

`data/m3_sample_planbus.mat` - a known-good real result (real M1 tracks,
real M2 predictions, real village-road geometry, ego at t=5.5s, goal 15m
ahead): 61 waypoints, 15.1m path, 0 direction switches. Useful for telling
"integration broke something" apart from "the planner's behavior
legitimately changed."