# SIH 2026 PS37 — Interface Contract
**Owner:** M6 (Integration Lead) | **Status as of:** Day 2 | **This is the single source of truth — do not maintain a second copy elsewhere.**

Any change to a field name, type, or size below requires updating this file AND notifying all 6 members. Do not silently diverge.

---

## 1. Perception → downstream (M1 owner)

Bus: `SihPerceptionBus` / `SihTrackBus` (defined in `buses/sihCreateBuses.m`)

- `class` — **enum** `AgentClass` (`buses/AgentClass.m`), NOT a string. Simulink buses cannot carry char arrays.
- Track list is **fixed-length**: `tracks[40]` + `num_tracks` (uint32) + per-track `valid` flag.
  **Always loop `1:num_tracks`, never `1:numel(tracks)`.**
  (Raised from 20→40 by M1 — fixed a real bug where 222/248 frames were silently failing to
  initiate new tracks at the old size.)
- Frame: ego-vehicle, ISO 8855 (x forward, y left, θ CCW).
- `covariance` = 4×4 over `[x, y, heading, velocity]`, already Jacobian-transformed by M1 from the
  tracker's internal `[x vx y vy z vz]` state. **Do not re-derive — use as-is.**
- Ego pose published separately via `SihEgoBus` (world frame, 5 fields incl. `Timestamp`) for anyone
  needing a transform.

**Known limitation (logged, not blocking):** M1 running `multiObjectTracker` (no Sensor Fusion Toolbox
available), not `trackerJPDA`. ~2.43 false tracks/frame near ego. Weakest on closely-spaced crossing
agents — expect this to show up in market/intersection scenarios.

**Fixed Day 2 (real bug, verified):** `Prediction/sihCreateBuses.m`, `Prediction/AgentClass.m`,
`Prediction/sihConfig.m` were stale duplicates silently SHADOWING the real `buses/sihCreateBuses.m`
and `perception/sihConfig.m` on the MATLAB path — confirmed via `which -all sihCreateBuses`, which
caused `MaxTracks` to silently resolve to 20 instead of 40 on any machine where `Prediction/` came
first on the path. All three deleted. If you see `MaxTracks=20` unexpectedly, run `which -all
sihCreateBuses` and check for shadowing before assuming it's a missing push.

## 2. Prediction → GlobalPlanner (M2 owner)

Bus: `SihPredictionBus` — flat per-agent shape, array size set on the block's output **port**
(Ports and Data Manager), NOT wrapped in a container bus. **Confirmed final Day 2** — M2 attempted
a container-bus wrapper (agents[]/num_agents/timestamp) but hit a real codegen limitation:
MATLAB Function blocks cannot construct a struct-array-valued field of a struct via any
construction order. Reverted to the flat shape, which was M2's original working design.

- Per-agent fields: `id` (uint32), `valid` (boolean), `predicted_positions[N x 2]` (double),
  `uncertainty_radius[N x 1]` (double).
- **No count/valid-total field** — consumers must loop all array slots checking `.valid`.
  Known inconsistency vs. the `num_tracks` convention elsewhere; planned follow-up post-Phase 1,
  not a blocker.
- **No batch timestamp** on this bus.
- Horizon length: currently **N=20** (matches M6's original placeholder value, kept by M2 when
  reverting the container-bus attempt) — **verify with M2 whether this is intentional or should be
  N=10 (M2's originally-stated real/tuned value) before treating as final.**

GlobalPlanner's prediction input port should be typed directly as `Bus: SihPredictedAgentBus` with a
fixed array dimension (40, matching `SihTrackBus`'s `MaxTracks`) set on the port — same pattern M2
uses on their output. No wrapper bus needed.

## 3. GlobalPlanner → Control (M3 owner)

Bus: `SihPlanBus` (defined in `control/createM4BusObjects.m` — **M4's file is authoritative**, not
duplicated elsewhere)

- `Waypoints` — `double [5000 x 4]`, columns `[x y theta v_signed]`, **world frame**.
- `NumWaypoints` — `uint32`. Rows beyond this are zero padding — always gate reads on this, not array size.
- `Directions` — `double [5000 x 1]`. Toolbox natively returns int8; cast to double in
  `assignVelocityProfile.m`, stays double through `packPlanBus.m`. M4's NMPC assumes double — this is
  final, not a placeholder.
- `SeqNum` — `uint32`, increments per new plan (M4 uses this to detect a fresh plan).
- `GenerationTimestamp`, `MapTimestamp`, `MapAgeAtPlan_s` — all `double`, on `navClock.m`'s **session-local
  monotonic clock**, NOT wall-clock. Do not compare against `posixtime`/`datetime`.
- **CONFIRMED Day 1 (M3):** `MAX_WAYPOINTS = 5000` matches `cfg.Bus.MaxWaypoints` /
  `cfg.Plan.MaxNumPathStates` in `planner_config.m` — locked, both sides agree.
- **NEW Day 2, CONFIRMED PUSHED (commit `59d1847`):** 8th field `PlannerInfeasible` (boolean) added.
  Set by M3 from the internal `IsPathFound`/`FailureReason` when packing the bus. Consumed by M5's
  Stateflow chart (verified: `PlannerInfeasible=true` correctly triggers `driving_mode=STOP`) via
  `extractPlannerInfeasible.m`. **Behavior for Control still needs group sign-off** — see Section 6.

**Footprint / collision-checking — RESOLVED Day 2, supersedes the Day 1 "single-disc + audit-only"
decision (kept below for history):**
M3 built and verified real 3-circle collision checking. Tested against all 5 scenarios: Urban
Intersection, Highway Merge, Dense Market, Cattle Crossing all **pass cleanly**. Village Road:
M5 widened the road 5.0m→5.2m, M3 re-ran `checkPathFootprint.m` and **confirmed it now clears 1.5m**
at the tight curve. All three previously-open sub-questions now closed, confirmed via
`planner/buildThreeCircleCollisionChecker.m`:
- **Architecture:** the 3-circle check gates the Hybrid A* search via `inflationCollisionChecker`
  wrapped in `validatorVehicleCostmap` — the toolbox-sanctioned path already permitted by the Day 1
  lock (never a *raw* `vehicleCostmap` directly, which this correctly avoids). No lock violation.
- **Reconciliation with M4's MPC footprint:** M3's circle offsets (`[-0.1167, 1.450, 3.0167]` m from
  rear axle) are the SAME numbers M4's independently-proposed 3-disc MPC footprint used — confirmed
  not two separate approximations. `buildThreeCircleCollisionChecker.m` derives its placements
  directly from `cfg.Foot.CircleOffsets_m` (the same config `checkPathFootprint.m` audits against),
  with an assertion guarding against the two ever silently drifting apart in future.
- **Village Road:** confirmed clear at 1.5m after the road widening.

*(Original Day 1 decision, for history — single-disc footprint (2.52m) undercovers the true vehicle
nose by 1.385m; audit showed 0/223 true-body collisions on placeholder maps; decision was
single-disc + audit-only given multi-circle wasn't natively supported by the Navigation Toolbox
validator. Fully superseded by the working, verified 3-circle implementation above.)*

**Status Day 2:** GlobalPlanner is a confirmed dummy stub in the top-level model. Class-enum blocker
resolved (`loadM1RealPerception.m` verified to genuinely consume `AgentClass` enum objects;
`generateFakeTracksAndPredictions.m`, the file still using plain strings, confirmed dead code with
zero live callers — **deleted by M3, confirmed via this pull**). Variable-length struct blocker: M3
confirmed three functions still need conversion (`buildRealOccupancyMap.m`, `loadM2RealPredictions.m`,
`computeM1FirstSeenTimes.m` using `containers.Map`) — **status of that conversion not yet
reconfirmed since M3's last update; do not swap GlobalPlanner from stub to real until confirmed.**

## 4. Ego state (two buses — legitimately different, not duplicates)

- **`SihEgoBus`** (M1 owner, `buses/sihCreateBuses.m`) — perception-published world pose, from sensing.
  5 fields: `x`, `y`, `yaw`, `velocity`, `Timestamp` (simulation time, not wall-clock).
- **`SihEgoStateBus`** — **DELETED Day 2**, confirmed via `sihDefineBuses()` output
  (`SihEgoStateBus : DELETED (was duplicate of SihEgoBus)`). `SihEgoBus` is now the single canonical
  ego bus.
- **`M4_VehicleDynamics.slx` now exists and is wired into the top-level model** — real plant with
  `steering_angle`/`acceleration` inputs, `ego_state`/`actor_pose` outputs. This appears to resolve
  the long-standing reconciliation question, but the exact wiring hasn't been independently traced
  field-by-field yet — treat as likely resolved, not fully confirmed.
- `scenario_decision_logic/scenarios/egoBusToScenarioPose.m` (new) converts `SihEgoBus` →
  Automated Driving Toolbox `ActorPose` for M5's Scenario Reader. **Open question sent to M1/M5:**
  this hardcodes `egoPose.ActorID = uint32(1)`, while `perception/sihCreateRealSensors.m` explicitly
  excludes ego by dynamic `ActorID` match, not by assuming index 1. Needs confirming the ego actor is
  guaranteed ActorID=1 in every scenario M5 builds, or this could silently misassign ego's pose.

## 5. Stateflow → Control (M5 owner, M4-only consumer)

Bus: `SihDrivingModeBus` (defined in `buses/sihDefineBuses.m`, M6-owned)

- `driving_mode` — enum `DrivingMode` (`buses/DrivingMode.m`): CRUISE / CAUTIOUS / YIELD / STOP.
- `timestamp` — double.
- **Consumed ONLY by M4** — tightens NMPC constraints, caps velocity profiler speed limit. Does NOT
  touch the occupancy map / inflation radii.
- `getDefaultValue()` on the enum **must return STOP** (fail-safe on a dropped/uninitialized signal).
  **Do not change this to CRUISE for convenience** — M4 was explicit on this.
- Values are severity-ordered. Escalation to a more severe mode should be immediate; downgrade to a less
  severe mode wants ~0.5s dwell time, or the speed cap will visibly surge/bounce.
- Concrete values (`control/drivingModeParams.m`, M4-owned, easy to retune):
  CRUISE 30 km/h / CAUTIOUS 15 / YIELD 5 / STOP 0. Obstacle margins: 0.5 / 1.0 / 1.5 / 2.0 m.

**`agent_density` / `risk_zone_high_risk_agent` — RESOLVED, no new bus needed.** Computed internally
by DecisionLogic's `computeDecisionSignals.m`, consuming M1's existing `SihPerceptionBus.tracks`/
`num_tracks` directly (position + class, current frame — not predictive). Verified correct.
Zone radii (`DENSITY_ZONE_RADIUS_M=30`, `RISK_ZONE_RADIUS_M=15`) are placeholders for Phase 1 tuning.

**`planner_infeasible` — RESOLVED Day 2.** `SihPlanBus.PlannerInfeasible` (boolean), confirmed pushed
(commit `59d1847`). Wired into the chart via `extractPlannerInfeasible.m`; M5 reports
`PlannerInfeasible=true` correctly triggers `driving_mode=STOP`, tested end-to-end.

**Status Day 2:** DecisionLogic still a confirmed dummy stub in the top-level model, though M5 reports
the real chart is fully wired and tested standalone (`computeDecisionSignals.m` and
`extractPlannerInfeasible.m` both confirmed integrated, dwell-time hysteresis added, repointed to
canonical `DrivingMode.m`). Merge into `sih_top_model.slx` not yet done — M5 has
`scenario_decision_logic/MERGE_CHECKLIST.md` ready. **`computeDecisionSignals.m`'s wiring into the
live chart's guard conditions has not been independently verified by M6** (not visible via text
search of the build script, since it may live inside the compiled `.slx` chart itself) — get direct
confirmation (screenshot of the actual guard condition) before merging.

## 6. Control → Vehicle (M4 owner)

Bus: `SihControlCmdBus` (defined in `control/createM4BusObjects.m`)

- `steering_angle` (double, rad, front wheel)
- `acceleration` (double, m/s², longitudinal)
- `speedCap_mps` (double) — **NOT an actuator command.** Derived from `driving_mode`, feeds BACK
  UPSTREAM to M3's `assignVelocityProfile` as its `externalSpeedCap_mps` argument.
  **This is a genuine feedback loop, not on the original architecture diagram.**
  Not yet wired in the top-level model (GlobalPlanner is still a stub — nothing real to feed).
  When GlobalPlanner goes real: add a Memory/unit-delay block on this signal so mode changes latch into
  the NEXT replan cycle, rather than creating a same-step algebraic loop between subsystems running at
  different rates.

**`PlannerInfeasible` behavior — LOCKED, signed off by M6 (Day 2):** When `PlannerInfeasible=true`, Control forces `driving_mode` toward STOP immediately and unconditionally — no debounce, overriding any other state. This is kept structurally separate from `planFreshnessGuard`'s ordinary staleness-hold logic (different failure category — a stale-but-valid plan vs. no plan at all). Recovery requires BOTH `planner_infeasible` clearing AND Control's own freshness check passing (`SeqNum` incremented, `MapAgeAtPlan_s` low) — the flag clearing alone is not sufficient. Recovery target is NOT hardcoded to CRUISE — falls through to whatever DecisionLogic's normal `agent_density`/`risk_zone_high_risk_agent` logic computes for the current state. M4 confirmed the STOP speed cap settles smoothly under NMPC (max residual |v|=0.02 m/s, not a brake-slam). **Still open:** whether a Control-side STOP override gets reflected back onto `SihDrivingModeBus` for M5/logging visibility, or stays purely internal to Control — unconfirmed, needs a decision.

**Status Day 2:** Control is REAL — wired via a `Model Reference` block to M4's `control/M4_Control.slx`
(not a nested Subsystem, deliberately, since `.slx` is binary and git can't merge it — keeps M4's file
and the top-level model as separate files nobody collides on). Port order:
1=`plan` (`SihPlanBus`), 2=`ego_state` (`SihEgoStateBus`), 3=`driving_mode` (`DrivingMode` enum).
Output 1=`cmd` (`SihControlCmdBus`). All ports nonvirtual (required for Model Reference).

The Simulink block itself is a compiled stub with correct types (constant output); the real NMPC
algorithm is validated in plain MATLAB (`control/setup_m4_day1.m`) but not yet ported into the block
(blocked on codegen bounds for variable-size arrays gated on `NumWaypoints`).

**KNOWN GAP, confirmed Day 2:** neither M6 nor M4 has MPC Toolbox installed — the full top-level model
cannot compile/run end-to-end on either machine (`Failed to load library 'mpclib'`). Unclear whether
this affects other team members too. **Needs a real decision, revisiting the Day 1 fallback options
(licence request / hand-rolled fmincon MPC / Pure Pursuit+PID) — this has been deferred since Day 1
without a final resolution, and now confirmed to block at least 2 of 6 team members from running the
complete model locally.**

**Verified Day 1: top-level model (`model/sih_top_model.slx`) ran end-to-end, 0 errors, 0 warnings**,
before `M4_VehicleDynamics.slx` was added — that specific verification is now stale given the model's
structure has changed since (VehicleDynamics block, bus selector added). Needs re-verification once
the mpclib gap is resolved on some machine that has the toolbox.

## 7. Vehicle model (locked, all members)

Kinematic bicycle: wheelbase 2.5m, length 4.7m, width 1.8m, rear overhang 0.9m, max steer 35°.
`MinTurningRadius` = 1.15 × (L/tan(δ)) = **4.10m** (kinematic value 3.57m + 15% safety margin — a chosen
safety margin, not a hard derivation). M3's planning radius (4.10m) is comfortably outside M4's physical
limit (3.57m) — any path M3 hands to M4 is drivable by construction.

## 8. Sample time / rates

Top-level model: fixed-step discrete, base rate **0.1s**, matching M4's Control `Ts` directly (M4's
explicit Day-1 request — avoids inserting rate transitions around their block). GlobalPlanner will
replan asynchronously at an eventual 2–5Hz target (Phase 2) — a rate transition will be needed at that
boundary once GlobalPlanner goes real. Perception/Prediction/DecisionLogic rates not yet pinned — Day 1
stubs run at the base rate.

## 9. Folder convention (repo-wide)

**Flat, no nesting**: `perception/`, `planner/`, `control/`, `buses/`, `data/`, `docs/`, `logs/`,
`metrics/`, `model/`. Two known exceptions, not yet fixed: `Prediction/` (capital P, M2's) and
`scenario_decision_logic/` (M5's, expected `decision/`). Low priority, flagged.

## 10. Scenarios (M5 owner) — RESOLVED

- ~~M5's RoadRunner requirement~~ — **RESOLVED (2026-08-29).** All 5 scenarios built in
  Driving Scenario Designer, RoadRunner dropped entirely. Village road + urban intersection
  (the 2 PS-required scenes) get extra visual/behavioral detail in DSD instead of a separate
  tool. Confirmed directly by M6 with full decision history (5 documented back-and-forths,
  final confirmation 2026-08-29). Technical report must document this substitution and
  reasoning explicitly, since it deviates from the PS's literal wording.

## 11. Open items — genuinely unresolved as of Day 2

- **M2 (Prediction)** — flat bus shape confirmed final; N=20 vs N=10 horizon needs confirming as
  intentional; no count field is a known, deferred inconsistency, not a blocker.
- **M3 (GlobalPlanner)** — footprint architecture fully resolved. Variable-length struct conversion
  status needs reconfirming (last known: 3 functions still need it, timing not yet given).
- **`PlannerInfeasible` field** — pushed and verified (bus definition: commit `59d1847`; packing logic: commit `bd68019`). Behavior spec LOCKED and written into Section 6 (Day 2). Still open: whether a Control-side STOP override reflects back onto `SihDrivingModeBus` for M5/logging.
- **M4/M6 — MPC Toolbox gap** — confirmed blocking at least 2 of 6 members from running the full
  model; Day 1 fallback decision never finalized, needs revisiting now that impact is confirmed wider.
- **M5 (DecisionLogic)** — chart reportedly fully wired and tested standalone; merge into top-level
  model not yet done; `computeDecisionSignals.m`'s live-chart wiring not independently verified by M6.
- `speedCap_mps` feedback loop (Section 6) — not wired, pending GlobalPlanner going real.
- **`SihEgoBus`/`egoBusToScenarioPose.m` ActorID=1 hardcoding** — question sent to M1/M5, unconfirmed
  whether this is always safe or could silently misassign ego's pose in some scenario configurations.
- **Duplicate/stale files in `Prediction/`** — RESOLVED Day 2, real shadowing bug found and fixed
  (see Section 1).