\# SIH 2026 PS37 — Interface Contract

\*\*Owner:\*\* M6 (Integration Lead) | \*\*Status as of:\*\* Day 1 | \*\*This is the single source of truth — do not maintain a second copy elsewhere.\*\*



Any change to a field name, type, or size below requires updating this file AND notifying all 6 members. Do not silently diverge.



\---



\## 1. Perception → downstream (M1 owner)



Bus: `SihPerceptionBus` / `SihTrackBus` (defined in `buses/sihCreateBuses.m`)



\- `class` — \*\*enum\*\* `AgentClass` (`buses/AgentClass.m`), NOT a string. Simulink buses cannot carry char arrays.

\- Track list is \*\*fixed-length\*\*: `tracks\[20]` + `num\_tracks` (uint32) + per-track `valid` flag.

&#x20; \*\*Always loop `1:num\_tracks`, never `1:numel(tracks)`.\*\*

\- Frame: ego-vehicle, ISO 8855 (x forward, y left, θ CCW).

\- `covariance` = 4×4 over `\[x, y, heading, velocity]`, already Jacobian-transformed by M1 from the

&#x20; tracker's internal `\[x vx y vy z vz]` state. \*\*Do not re-derive — use as-is.\*\*

\- Ego pose published separately via `SihEgoBus` (world frame) for anyone needing a transform.



\*\*Known limitation (logged, not blocking):\*\* M1 running `multiObjectTracker` (no Sensor Fusion Toolbox

available), not `trackerJPDA`. \~2.43 false tracks/frame near ego. Weakest on closely-spaced crossing

agents — expect this to show up in market/intersection scenarios.



\## 2. Prediction → GlobalPlanner (M2 owner)



Bus: `SihPredictionBus` (defined in `buses/sihDefineBuses.m`, M6-owned since M2 has not yet delivered

their own script)



\- Fixed-length: `agents\[20]` (mirrors `SihTrackBus`'s `MaxTracks`) + `num\_agents` (uint32).

\- Per agent: `id` (uint32), `predicted\_positions\[20x2]` (double, `\[x y]` per horizon step),

&#x20; `uncertainty\_radius\[20]` (double), `valid` (boolean).

\- \*\*STATUS: UNCONFIRMED.\*\* M2 has not reported Day 1 status, toolbox check, or predictor frame

&#x20; convention. Frame is assumed ego (matching M1) but not confirmed by M2 directly.



\## 3. GlobalPlanner → Control (M3 owner)



Bus: `SihPlanBus` (defined in `control/createM4BusObjects.m` — \*\*M4's file is authoritative\*\*, not

duplicated elsewhere)



\- `Waypoints` — `double \[5000 x 4]`, columns `\[x y theta v\_signed]`, \*\*world frame\*\*.

\- `NumWaypoints` — `uint32`. Rows beyond this are zero padding — always gate reads on this, not array size.

\- `Directions` — `double \[5000 x 1]`. Toolbox natively returns int8; cast to double in

&#x20; `assignVelocityProfile.m`, stays double through `packPlanBus.m`. M4's NMPC assumes double — this is

&#x20; final, not a placeholder.

\- `SeqNum` — `uint32`, increments per new plan (M4 uses this to detect a fresh plan).

\- `GenerationTimestamp`, `MapTimestamp`, `MapAgeAtPlan\_s` — all `double`, on `navClock.m`'s \*\*session-local

&#x20; monotonic clock\*\*, NOT wall-clock. Do not compare against `posixtime`/`datetime`.

\- \*\*CONFIRMED Day 1 (M3):\*\* `MAX\_WAYPOINTS = 5000` matches `cfg.Bus.MaxWaypoints` /

&#x20; `cfg.Plan.MaxNumPathStates` in `planner\_config.m` — locked, both sides agree.



\*\*Known limitation, DECIDED (conditional):\*\* M3's planner uses a single-disc footprint (2.52m radius)

for collision checking; true vehicle body (4.7×1.8m) undercovers the nose by 1.385m

(`hypot(3.8,0.9) - 2.52`). Audit (`checkPathFootprint.m`) shows 0/223 true-body collisions on every

placeholder map tested. \*\*Decision: keep single-disc + audit-only at the planner level\*\* — switching to

multi-circle isn't natively supported by the Navigation Toolbox validator (real engineering work, not a

config flip). Mitigating factor: M4's MPC independently uses a validated 3-disc footprint

(r=1.193m at x=\[-0.117, 1.450, 3.017] from rear axle) for real-time obstacle avoidance, so the planner's

optimism is not the last line of defense. \*\*Revisit trigger:\*\* any true-body collision flagged by

`checkPathFootprint.m` during Phase 2 closed-loop testing, especially dense-market/narrow-lane scenarios.



\*\*Status Day 1:\*\* GlobalPlanner is a confirmed dummy stub in the top-level model. M3's real pipeline is

logically solid but not yet Simulink-safe (variable-length structs, string-typed `class` field) —

M3 is converting internals to fixed-size/enum-typed today, targeting Day 2–3 wiring.



\## 4. Ego state (two buses — legitimately different, not duplicates)



\- \*\*`SihEgoBus`\*\* (M1 owner, `buses/sihCreateBuses.m`) — perception-published world pose, from sensing.

\- \*\*`SihEgoStateBus`\*\* (M4 owner, `control/createM4BusObjects.m`) — vehicle-dynamics plant's own state

&#x20; feedback to the controller: `X`, `Y` (rear-axle, world frame), `psi` (heading, rad, CCW+),

&#x20; `v` (signed speed, m/s).

\- \*\*Do not assume these are interchangeable\*\* despite similar names/fields — different purpose, different

&#x20; producer.

\- \*\*Current top-level model wiring (interim, Day 1):\*\* Control's `ego\_state` input is fed by a dedicated

&#x20; zero-init stub (`EgoStateConst`), NOT by Perception's real `EgoOut`. Perception's `SihEgoBus` output

&#x20; currently goes unused in the model. This needs real reconciliation once M4's vehicle dynamics plant

&#x20; exists and actually publishes something for Control to consume.



\## 5. Stateflow → Control (M5 owner, M4-only consumer)



Bus: `SihDrivingModeBus` (defined in `buses/sihDefineBuses.m`, M6-owned)



\- `driving\_mode` — enum `DrivingMode` (`buses/DrivingMode.m`): CRUISE / CAUTIOUS / YIELD / STOP.

\- `timestamp` — double.

\- \*\*Consumed ONLY by M4\*\* — tightens NMPC constraints, caps velocity profiler speed limit. Does NOT

&#x20; touch the occupancy map / inflation radii.

\- `getDefaultValue()` on the enum \*\*must return STOP\*\* (fail-safe on a dropped/uninitialized signal).

&#x20; \*\*Do not change this to CRUISE for convenience\*\* — M4 was explicit on this.

\- Values are severity-ordered. Escalation to a more severe mode should be immediate; downgrade to a less

&#x20; severe mode wants \~0.5s dwell time, or the speed cap will visibly surge/bounce.

\- Concrete values (`control/drivingModeParams.m`, M4-owned, easy to retune):

&#x20; CRUISE 30 km/h / CAUTIOUS 15 / YIELD 5 / STOP 0. Obstacle margins: 0.5 / 1.0 / 1.5 / 2.0 m.



\*\*Status Day 1:\*\* DecisionLogic is a confirmed dummy stub. M5's real Stateflow chart status and exact

file path in the repo are \*\*unconfirmed\*\* as of Day 1.



\## 6. Control → Vehicle (M4 owner)



Bus: `SihControlCmdBus` (defined in `control/createM4BusObjects.m`)



\- `steering\_angle` (double, rad, front wheel)

\- `acceleration` (double, m/s², longitudinal)

\- `speedCap\_mps` (double) — \*\*NOT an actuator command.\*\* Derived from `driving\_mode`, feeds BACK

&#x20; UPSTREAM to M3's `assignVelocityProfile` as its `externalSpeedCap\_mps` argument.

&#x20; \*\*This is a genuine feedback loop, not on the original architecture diagram.\*\*

&#x20; Not yet wired in the top-level model (GlobalPlanner is still a stub — nothing real to feed).

&#x20; When GlobalPlanner goes real: add a Memory/unit-delay block on this signal so mode changes latch into

&#x20; the NEXT replan cycle, rather than creating a same-step algebraic loop between subsystems running at

&#x20; different rates.



\*\*Status Day 1:\*\* Control is REAL — wired via a `Model Reference` block to M4's `control/M4\_Control.slx`

(not a nested Subsystem, deliberately, since `.slx` is binary and git can't merge it — keeps M4's file

and the top-level model as separate files nobody collides on). Port order:

1=`plan` (`SihPlanBus`), 2=`ego\_state` (`SihEgoStateBus`), 3=`driving\_mode` (`DrivingMode` enum).

Output 1=`cmd` (`SihControlCmdBus`). All ports nonvirtual (required for Model Reference).



The Simulink block itself is a compiled stub with correct types (constant output); the real NMPC

algorithm is validated in plain MATLAB (`control/setup\_m4\_day1.m`) but not yet ported into the block

(blocked on codegen bounds for variable-size arrays gated on `NumWaypoints`).



\## 7. Vehicle model (locked, all members)



Kinematic bicycle: wheelbase 2.5m, length 4.7m, width 1.8m, rear overhang 0.9m, max steer 35°.

`MinTurningRadius` = 1.15 × (L/tan(δ)) = \*\*4.10m\*\* (kinematic value 3.57m + 15% safety margin — a chosen

safety margin, not a hard derivation). M3's planning radius (4.10m) is comfortably outside M4's physical

limit (3.57m) — any path M3 hands to M4 is drivable by construction.



\## 8. Sample time / rates



Top-level model: fixed-step discrete, base rate \*\*0.1s\*\*, matching M4's Control `Ts` directly (M4's

explicit Day-1 request — avoids inserting rate transitions around their block). GlobalPlanner will

replan asynchronously at an eventual 2–5Hz target (Phase 2) — a rate transition will be needed at that

boundary once GlobalPlanner goes real. Perception/Prediction/DecisionLogic rates not yet pinned — Day 1

stubs run at the base rate.



\## 9. Folder convention (repo-wide)



\*\*Flat, no nesting\*\*: `perception/`, `planner/`, `control/`, `buses/`, `data/`, `docs/`, `logs/`,

`metrics/`, `model/`. `scenario\_decision\_logic/` (M5) is a known naming inconsistency, not yet resolved

— flagged, not urgent.



\## 10. Open items — genuinely unresolved as of Day 1



\- \*\*M2 (Prediction)\*\* has not reported any Day 1 status. Highest-priority open gap.

\- \*\*M5's RoadRunner requirement\*\* — original split doc required ≥2 RoadRunner scenes as a "never cut"

&#x20; item. M5 claimed this was dropped ("all 5 in DSD") citing an "updated task split" that could not be

&#x20; verified against any authoritative source. \*\*Multiple unverified documents have since claimed this is

&#x20; resolved — none confirmed against the real repo or a genuine team decision. Treat as still open until

&#x20; independently confirmed by more than one person.\*\*

\- `speedCap\_mps` feedback loop (Section 6) — not wired, pending GlobalPlanner going real.

\- `SihEgoBus` vs `SihEgoStateBus` real reconciliation (Section 4) — pending M4's vehicle dynamics plant.

