# M1 — Perception & Sensor Fusion Lead

Day-1 (Phase 0) deliverable for SIH 2026 PS37. Everything below runs today, on dummy detections, and emits the locked interface contract.

## Run it

```matlab
cd sih_m1_perception
results = runPerceptionStub();                 % with bird's-eye visualization
results = runPerceptionStub('Export', true);   % also writes the .mat for M6
```

You should get a bird's-eye plot with three sensor coverage wedges, raw detections, and fused tracks labelled `id:Class` — plus a printed summary with position RMSE, classification accuracy, and track yield.

## What each file does

| File | Purpose |
|---|---|
| `AgentClass.m` | The 6-class enum. This is the `class` field of the contract. |
| `sihConfig.m` | Every tunable parameter. Phase-1 tuning = edit this one file. |
| `sihCreateBuses.m` | Builds `SihTrackBus`, `SihPerceptionBus`, `SihEgoBus`. **Give these to M6 today.** |
| `sihBuildScenario.m` | Your own mixed-traffic test scenario, so you aren't blocked on M5. |
| `sihDummyDetections.m` | Stub camera/radar/LiDAR. Drop-in replaceable on Day 2. |
| `sihCreateTracker.m` | trackerJPDA, with automatic fallback to trackerGNN / multiObjectTracker. |
| `sihClassVoter.m` | Fuses per-detection class labels into one stable class per track. |
| `sihTracksToContract.m` | EKF state → contract format, including the covariance transform. |
| `runPerceptionStub.m` | Main loop + ground-truth validation. |
| `sihExportForSimulink.m` | Exports bus-format data so M6 can wire the model today. |

---

## Three things to raise with the team TODAY

The contract doc says don't change field names without notifying all six of you. These are exactly that — none of them are optional, all three are cheap now and expensive on Day 5.

**1. `class` cannot be a string.** A Simulink Bus can't carry a char array or string. It has to be an enumeration or a `uint8`. `AgentClass.m` is the enum — send it round and have everyone reference `AgentClass.Pedestrian` rather than `'pedestrian'`. This matters most to M3 (risk weighting in the costmap) and M5 (risk-zone guard conditions in Stateflow).

**2. "Per timestep struct array" needs a fixed size.** Simulink buses are fixed-width, so a variable-length list of tracks isn't expressible. The contract becomes:

```
tracks[20]  — fixed array of the track struct
num_tracks  — uint32, how many are real
timestamp   — double, seconds
```

Each track also carries `valid`. Consumers loop `1:num_tracks`. If M2 or M3 write code assuming a variable-length array, it will break at the bus boundary and it won't be obvious why.

**3. Frame and covariance need pinning down.** Two decisions baked into this code, both of which the team should ratify or override:

- **Frame:** ego-vehicle, ISO 8855 (x forward, y left, θ CCW). Perception is naturally ego-centric; M3's costmap may want world. Ego pose in world is published on a separate `SihEgoBus` so whoever needs the transform can do it, without touching the perception contract.
- **`covariance` = 4×4 over `[x, y, heading, velocity]`** — it mirrors the exposed fields exactly, and it's what M2 actually needs to size uncertainty cones.

On that last point, one thing worth flagging in the standup, because it's the most likely silent bug in this module: the EKF's state is `[x vx y vy z vz]`, and heading and speed are *nonlinear* functions of it. You can't slice the covariance matrix — it has to go through the Jacobian (`P_c = J·P·Jᵀ`, done in `sihTracksToContract.m`). If someone slices it instead, M2's uncertainty cones will be confidently wrong and nothing will visibly fail.

---

## Licence warning worth checking before you promise anything

`trackerJPDA` and `trackerGNN` are in **Sensor Fusion and Tracking Toolbox**, not Automated Driving Toolbox. ADT only ships `multiObjectTracker`. If your machines have ADT alone, JPDA won't exist. `sihCreateTracker.m` detects this and falls back automatically, so nothing breaks either way — but check now:

```matlab
ver          % look for "Sensor Fusion and Tracking Toolbox"
which trackerJPDA
```

JPDA is worth having. GNN forces a hard one-to-one detection-to-track assignment, which is precisely what fails when two pedestrians cross close together or a two-wheeler squeezes past an auto-rickshaw — the two most common events in your market and intersection scenarios. If you don't have the licence, say so early; it's a real constraint on how well the market scenario will behave.

---

## Why not a MATLAB Function block yet

MATLAB Function blocks require code generation. `trackerJPDA` plus a `containers.Map` class voter is not codegen-clean, and losing Day 1 to codegen errors is how a 9-day timeline dies.

So today: run perception in MATLAB, export via `sihExportForSimulink`, and M6 drives the model with a `From Workspace` block typed as `SihPerceptionBus`. M2, M3 and M6 are all unblocked immediately with *real* fused-track data, not hand-written dummies.

Day 2, migrate in this order: (1) the **Multi-Object Tracker Simulink block** from the ADT/SFTT library — purpose-built, no codegen pain; (2) a MATLAB System block; (3) MATLAB Function block with `coder.extrinsic`.

---

## Day 1 exit checklist

- [ ] `runPerceptionStub()` runs clean end to end
- [ ] Bus objects exist in base workspace; `SihTrackBus` / `SihPerceptionBus` / `SihEgoBus` sent to M6
- [ ] `AgentClass.m` sent to M2, M3, M5 (they all consume `class`)
- [ ] `m1_perception_day1.mat` handed to M6 for the top-level skeleton
- [ ] Three interface points above raised and agreed in writing
- [ ] Toolbox licence for `trackerJPDA` confirmed or flagged

## Day 2–4 (Phase 1) preview

Your Phase-1 task is swapping the stubs for real sensor models and tuning association. The swap is contained entirely inside `sihDummyDetections.m`:

- Camera → `visionDetectionGenerator`
- Radar → `drivingRadarDataGenerator`
- LiDAR → `lidarPointCloudGenerator` + clustering into bounding boxes

All three emit the same `objectDetection` cell array, so nothing downstream of that one function changes. The validation harness in `runPerceptionStub.m` (position RMSE, classification accuracy, track yield) already gives you the before/after numbers you'll need for the Phase-1 checkpoint and the report section on Day 8.
