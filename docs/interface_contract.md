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

## 2. Prediction → GlobalPlanner (M2 owner)

Bus: `SihPredictionBus` — **STATUS: STILL BEING RECONCILED, do not wire against this yet.**

M2's real script (`Prediction/createPredictionBusObjects.m`) defines a bus-array pattern: the bus
object itself is one agent's shape (`id`, `valid`, `predicted_positions[10x2]`,
`uncertainty_radius[10x1]`), with the fixed-size array (matching M1's `MaxTracks`) set on the block's
output **port** in Ports and Data Manager, not in the bus object file itself.

**Open questions, unconfirmed by M2:**
- No confirmed count/valid-total signal — unclear whether GlobalPlanner loops all slots checking
  `.valid`, or a companion count field exists elsewhere.
- No confirmed batch timestamp on prediction output.
- M2 is mid-fix syncing their array size from 20→40 to match M1's real `MaxTracks` change —
  **do not wire against this bus until M2 confirms the sync is pushed.**
- Horizon length N=10 explicitly NOT final — M2 expects to retune once M1's real Day 3 data lands.

`buses/sihDefineBuses.m` currently still has an M6-authored placeholder `SihPredictionBus`
(container-bus shape, structurally different from M2's real one) — **this needs deleting once M2's
open items above are answered**, replaced with a call to M2's `createPredictionBusObjects()`,
same pattern used for M1 and M4.

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

**Known limitation, DECIDED (conditional):** M3's planner uses a single-disc footprint (2.52m radius)
for collision checking; true vehicle body (4.7×1.8m) undercovers the nose by 1.385m
(`hypot(3.8,0.9) - 2.52`). Audit (`checkPathFootprint.m`) shows 0/223 true-body collisions on every
placeholder map tested. **Decision: keep single-disc + audit-only at the planner level** — switching to
multi-circle isn't natively supported by the Navigation Toolbox validator (real engineering work, not a
config flip). Mitigating factor: M4's MPC independently uses a validated 3-disc footprint
(r=1.193m at x=[-0.117, 1.450, 3.017] from rear axle) for real-time obstacle avoidance, so the planner's
optimism is not the last line of defense. **Revisit trigger:** any true-body collision flagged by
`checkPathFootprint.m` during Phase 2 closed-loop testing, especially dense-market/narrow-lane scenarios.

**Status Day 2:** GlobalPlanner is a confirmed dummy stub in the top-level model. M3's real pipeline was
not Simulink-safe as of Day 1 (variable-length structs, string-typed `class` field) — M3 was converting
internals to fixed-size/enum-typed, targeting Day 2–3 wiring. Real occupancy-map and perception-loading
files (`buildRealOccupancyMap.m`, `loadM1RealPerception.m`, `loadM2RealPredictions.m`, etc.) have since
appeared in the repo — **status of the Simulink-safety conversion not yet independently confirmed.**

## 4. Ego state (two buses — legitimately different, not duplicates)

- **`SihEgoBus`** (M1 owner, `buses/sihCreateBuses.m`) — perception-published world pose, from sensing.
  5 fields: `x`, `y`, `yaw`, `velocity`, `Timestamp` (simulation time, not wall-clock).
- **`SihEgoStateBus`** (M4 owner, `control/createM4BusObjects.m`) — vehicle-dynamics plant's own state
  feedback to the controller: `X`, `Y` (rear-axle, world frame), `psi` (heading, rad, CCW+),
  `v` (signed speed, m/s).
- **Do not assume these are interchangeable** despite similar names/fields — different purpose, different
  producer.
- **Current top-level model wiring (interim, Day 1):** Control's `ego_state` input is fed by a dedicated
  zero-init stub (`EgoStateConst`), NOT by Perception's real `EgoOut`. Perception's `SihEgoBus` output
  currently goes unused in the model. This needs real reconciliation once M4's vehicle dynamics plant
  exists and actually publishes something for Control to consume.

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

**Status Day 2:** DecisionLogic is a confirmed dummy stub. M5's real Stateflow chart file path in the
repo — `scenario_decision_logic/stateflow_stub/build_driving_mode_stateflow.m` and
`computeDecisionSignals.m` now exist in the repo, but explicit confirmation of the exact wiring/status
from M5 is still pending.

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

**Status Day 2:** Control is REAL — wired via a `Model Reference` block to M4's `control/M4_Control.slx`
(not a nested Subsystem, deliberately, since `.slx` is binary and git can't merge it — keeps M4's file
and the top-level model as separate files nobody collides on). Port order:
1=`plan` (`SihPlanBus`), 2=`ego_state` (`SihEgoStateBus`), 3=`driving_mode` (`DrivingMode` enum).
Output 1=`cmd` (`SihControlCmdBus`). All ports nonvirtual (required for Model Reference).

The Simulink block itself is a compiled stub with correct types (constant output); the real NMPC
algorithm is validated in plain MATLAB (`control/setup_m4_day1.m`) but not yet ported into the block
(blocked on codegen bounds for variable-size arrays gated on `NumWaypoints`).

**Verified: top-level model (`model/sih_top_model.slx`) runs end-to-end, 0 errors, 0 warnings**, with
Control real (Model Reference) and Perception/Prediction/GlobalPlanner/DecisionLogic as confirmed dummy
stubs. This is the Phase 0 exit checkpoint, met.

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

- **M2 (Prediction)** — bus shape partially clarified (Section 2), still waiting on count/timestamp
  confirmation and the MaxTracks 20→40 sync push before wiring in.
- **M3 (GlobalPlanner)** — Simulink-safety conversion status not independently confirmed since Day 1.
- **M5 (DecisionLogic)** — exact Stateflow chart file/wiring status not independently confirmed.
- `speedCap_mps` feedback loop (Section 6) — not wired, pending GlobalPlanner going real.
- `SihEgoBus` vs `SihEgoStateBus` real reconciliation (Section 4) — pending M4's vehicle dynamics plant.
- **Duplicate/stale files in `Prediction/`** (`sihCreateBuses.m`, `AgentClass.m`, `sihConfig.m`) — same
  risk class as fixed bugs earlier. Needs M2 to delete once Section 2's open items are resolved.