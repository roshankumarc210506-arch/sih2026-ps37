\# SIH 2026 PS37 — Day 1 (–2) Risk Log

\*\*Owner:\*\* M6 | Every item below is either open, or decided-with-a-condition. Nothing marked done here should be treated as permanently closed — re-check the trigger before assuming it's still true.



\---



\## Open — needs a person to answer



\### R1. M2's SihPredictionBus shape conflict

`buses/sihDefineBuses.m` defines `SihPredictionBus` as a top-level array wrapper

(`agents\[40] + num\_agents + timestamp`). `Prediction/createPredictionBusObjects.m`

defines `SihPredictionBus` as a single per-agent struct (`id, valid,

predicted\_positions\[10x2], uncertainty\_radius\[10x1]`), no array wrapper. These are

structurally incompatible, not just a naming difference. Horizon length also differs

(20 vs M2's real N=10). \*\*Message sent to M2, awaiting reply.\*\* Do not swap the stub

for M2's real script until this is resolved.



\### R2. M5's RoadRunner requirement

Original split doc listed ≥2 RoadRunner scenes as a "never cut" PS requirement. M5

reported building all 5 scenarios in Driving Scenario Designer only, citing an

"updated task split" that could not be independently verified. Multiple documents

since have claimed this is resolved, but none were confirmed against an actual team

decision or the real PS rules. \*\*Status: still genuinely open\*\* — needs direct,

independent confirmation from more than one team member, or a check against the

actual competition requirements, before treating as settled.



\### R3. speedCap\_mps feedback loop not wired

`SihControlCmdBus.speedCap\_mps` (M4's output) is meant to feed back upstream to

GlobalPlanner's `assignVelocityProfile` as `externalSpeedCap\_mps` — a genuine

feedback loop, not on the original architecture diagram. Not wired in the top-level

model yet (GlobalPlanner is still a stub). \*\*When GlobalPlanner goes real:\*\* add a

Memory/unit-delay block on this signal so mode changes latch into the next replan

cycle, not a same-step algebraic loop between subsystems running at different rates.



\---



\## Duplicate/stale files — same class of bug as the DrivingMode.m fix earlier today



\### R4. Prediction/ folder contains stale copies of canonical files

`Prediction/sihCreateBuses.m` — stale: missing `SihEgoBus.Timestamp` field (5th

field, added by M1 today), and still calls `sihConfig()` directly despite the

canonical file's explicit warning not to (path-resolution risk, confirmed real since

`perception/` and `Prediction/` both have a `sihConfig.m`).

`Prediction/AgentClass.m`, `Prediction/sihConfig.m` — not yet diffed against

canonical versions, same risk class. \*\*Action: confirm with M2 alongside R1, then

have M2 delete these three and reference `buses/`/`perception/` directly.\*\*

`Prediction/sihDefineBuses.m` — checked, confirmed byte-identical to

`buses/sihDefineBuses.m` as of the merge. Harmless, but should still be deleted for

hygiene once M2 is in the loop anyway.



\### R5. Prediction/ folder naming

Capital-P `Prediction/` doesn't match the flat lowercase convention used everywhere

else (`perception/`, `planner/`, `control/`, `buses/`). Low priority, cosmetic, but

will eventually confuse a case-sensitive script or command. Flag to M2 alongside R1/R4.



\### R6. scenario\_decision\_logic/ folder naming

Same category as R5 — M5's folder doesn't match the flat convention either

(expected something like `decision/`). Low priority, not urgent.



\---



\## Decided, with a condition to revisit



\### R7. Planner footprint fidelity

M3's planner uses a single-disc footprint (2.52m) for collision checking; true

vehicle body undercovers the nose by 1.385m. Audit shows 0/223 true-body collisions

on every placeholder map tested. \*\*Decision: keep single-disc + audit-only\*\* at the

planner level — multi-circle isn't natively supported by the Navigation Toolbox

validator, real engineering work. Mitigating factor: M4's MPC already uses a

validated 3-disc footprint independently for real-time obstacle avoidance.

\*\*Revisit trigger:\*\* any true-body collision flagged by `checkPathFootprint.m`

during Phase 2 closed-loop testing, especially dense-market/narrow-lane scenarios.



\### R8. MAX\_TRACKS sizing — fixed today, but shows the pattern to watch for

Was hardcoded to 20 in `sihDefineBuses.m`, silently out of sync with M1's real fix

to 40 (raised because 222/248 frames were losing track data at the old size — a real

bug, not tuning). \*\*Fixed 2026-08-29.\*\* Logged here as a pattern: any hardcoded

sizing constant duplicated across files (`MaxTracks`, `MAX\_WAYPOINTS`, horizon N)

needs a single owner, or it will silently drift exactly like this did.



\---



\## Known limitations — logged for the technical report, not blocking



\### R9. M1's tracker fallback

No Sensor Fusion Toolbox available → running `multiObjectTracker` instead of

`trackerJPDA`. \~2.43 false tracks/frame near ego. Weakest on closely-spaced crossing

agents — expect this to show up in market/intersection scenario testing.



\### R10. SihEgoBus vs SihEgoStateBus — real reconciliation still pending

Two buses, similar fields, genuinely different purposes (M1's is perception-published

world pose; M4's is the vehicle-dynamics plant's own state feedback). Current

top-level model wiring: Control's `ego\_state` input uses a dedicated zero-init stub,

NOT Perception's real `EgoOut` (which currently goes unused). \*\*Needs real

reconciliation once M4's vehicle dynamics plant exists and actually publishes

something for Control to consume from Perception/the plant.\*\*



\---



\## Genuinely unverified — do not cite as fact



\### R11. Claims from a parallel/duplicate Claude session

Several documents surfaced today (a "handoff doc," a "Day 2 revision" of the split

doc) made claims about work done, bugs fixed, and decisions made that could not be

verified against the real repo at the time they were presented. Some turned out to

be real once the actual `git ls-tree`/merge came through (M2's Prediction/ work,

M3's real planner files, M5's computeDecisionSignals.m); others (the specific

RoadRunner "resolution," the exact wording of some claimed fixes) remain unverified.

\*\*Standing rule for the rest of this project: verify any "done"/"confirmed" claim

against the actual repo contents before relaying it to anyone else as fact.\*\*

