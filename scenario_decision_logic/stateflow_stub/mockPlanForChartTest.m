%MOCKPLANFORCHARTTEST Mock SihPlanBus-shaped data for testing
%   extractPlannerInfeasible.m wiring into DrivingModeChart.
%
%   Matches the real bus shape in control/createM4BusObjects.m as of
%   commit 59d1847 (8 elements, PlannerInfeasible boolean appended last).
%
%   Creates two test cases in the base workspace:
%     mockPlan_feasible    - PlannerInfeasible = false -> chart should
%                            NOT force STOP from this signal alone
%     mockPlan_infeasible  - PlannerInfeasible = true  -> chart should
%                            transition to STOP immediately

MAX_WAYPOINTS = 5000;   % locked, per M3/M4 Day 1 agreement

% ---------- Case 1: feasible plan ----------
plan.Waypoints           = zeros(MAX_WAYPOINTS, 4);
plan.NumWaypoints        = uint32(10);   % a few real waypoints, rest padding
plan.Waypoints(1:10, :)  = [ (0:9)' , zeros(10,1), zeros(10,1), 5*ones(10,1) ];
plan.Directions          = ones(MAX_WAYPOINTS, 1);   % all forward
plan.SeqNum              = uint32(1);
plan.GenerationTimestamp = 0.0;
plan.MapTimestamp        = 0.0;
plan.MapAgeAtPlan_s      = 0.05;
plan.PlannerInfeasible   = false;

mockPlan_feasible = plan;

% ---------- Case 2: infeasible plan ----------
plan.PlannerInfeasible = true;   % everything else identical -- only this flips
mockPlan_infeasible = plan;

% ---------- quick sanity check when run directly ----------
fprintf('--- Case 1 (feasible) ---\n');
r1 = extractPlannerInfeasible(mockPlan_feasible);
fprintf('planner_infeasible=%d (expect 0)\n', r1);

fprintf('--- Case 2 (infeasible) ---\n');
r2 = extractPlannerInfeasible(mockPlan_infeasible);
fprintf('planner_infeasible=%d (expect 1)\n', r2);