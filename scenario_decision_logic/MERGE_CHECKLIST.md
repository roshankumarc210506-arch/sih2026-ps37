# Merge Checklist — DecisionLogic_Stub -> sih_top_model.slx
Owner: M5 | Status: pending M6 (shared model access + timing)

Everything below is mechanical -- no new decisions needed, just execution
once M6 says sih_top_model.slx is ready to receive the chart.

1. Copy DrivingModeChart block from DecisionLogic_Stub.slx into
   sih_top_model.slx (drag-copy or Stateflow's "Export chart" if copying
   across files loses formatting).

2. Delete the test scaffolding that stays behind in DecisionLogic_Stub.slx
   only -- do NOT carry these into sih_top_model.slx:
   - Constant blocks feeding mock tracks / num_tracks
     (mockTracks_pedestrianRisk / mockNumTracks_pedestrianRisk, etc.)
   - MyDisplay block
   - The "false" Constant standing in for planner_infeasible

3. In sih_top_model.slx, wire the chart's MATLAB Function block
   (decisionSignalsWrapper, calls computeDecisionSignals.m) inputs to
   the REAL SihPerceptionBus.tracks and SihPerceptionBus.num_tracks
   signals already present in the top-level model (from M1's Perception
   block/stub).

4. Wire planner_infeasible input via extractPlannerInfeasible.m,
   sourced from SihPlanBus.PlannerInfeasible -- confirm the field name
   matches whatever M3/M4/M6 finalize before wiring (see that file's
   header comment).

5. Wire driving_mode output into SihDrivingModeBus, feeding M4 ONLY --
   per the locked interface contract, Section 5. Does not touch the
   occupancy map / inflation radii.

6. Re-run equivalent test scenarios against REAL data once wired
   (not mocks) -- confirm CRUISE/CAUTIOUS/YIELD/STOP are all reachable
   under realistic track/plan data, same way mockTracksForChartTest.m
   validated the standalone version.

7. Once verified, commit from within sih_top_model.slx's repo location
   (this is M6's file -- coordinate on who commits vs. reviews).

Reference files already built and pushed, ready to reuse as-is in this
merge:
  scenario_decision_logic/stateflow_stub/computeDecisionSignals.m
  scenario_decision_logic/stateflow_stub/build_driving_mode_stateflow.m
  scenario_decision_logic/stateflow_stub/extractPlannerInfeasible.m
  scenario_decision_logic/stateflow_stub/mockTracksForChartTest.m (test-only, for step 6 comparison)