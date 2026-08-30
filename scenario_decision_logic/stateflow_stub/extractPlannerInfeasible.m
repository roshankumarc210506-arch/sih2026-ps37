function planner_infeasible = extractPlannerInfeasible(plan)
%EXTRACTPLANNERINFEASIBLE Extract the planner-infeasibility flag from
%   M3's plan bus, for wiring into DrivingModeChart's planner_infeasible
%   input.
%
%   STATUS: PENDING CONFIRMATION. M6 proposed adding PlannerInfeasible
%   as a boolean field directly on SihPlanBus (rather than a separate
%   bus, since it's plan metadata) -- sent to M3/M4 for confirmation.
%   This function assumes that proposal is accepted as-is. If the final
%   locked field name/location differs, update the one line below --
%   nothing else in the chart wiring needs to change.
%
%   Background (M3, Day 2): the underlying data already exists on every
%   plan cycle as IsPathFound/FailureReason inside the internal
%   planResult struct -- it was just never exposed on the locked bus.
%   This function is the boundary that exposes it to DecisionLogic.
%
%   Input:
%     plan - SihPlanBus struct/bus signal from M3.
%   Output:
%     planner_infeasible - boolean, true when M3's planner could not
%                           find a feasible path this cycle.

planner_infeasible = plan.PlannerInfeasible;   % UPDATE if field name changes on confirmation
end