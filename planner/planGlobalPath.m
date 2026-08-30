function planResult = planGlobalPath(omap, cfg, mapInfo, startPose, goalPose)
%PLANGLOBALPATH  Hybrid A* over SE(2) with Reeds-Shepp analytic expansion.
%
%   Returns geometry only: [x y theta] + directions. No velocity, no
%   per-waypoint timestamps - those are the profiler's job (Step 5).
%
%   MODE BRANCH (new - three-circle collision checking):
%   plannerHybridAStar's StateValidator property only accepts
%   validatorOccupancyMap or validatorVehicleCostmap (confirmed via
%   MathWorks docs - no custom nav.StateValidator subclass is accepted,
%   so a hand-rolled multi-point validator cannot be wired in here).
%   'single'      : omap is a binaryOccupancyMap (egoR already baked in
%                   by buildRealOccupancyMap.m) -> validatorOccupancyMap,
%                   exactly as before.
%   'threeCircle' : omap is a vehicleCostmap with a real
%                   InflationCollisionChecker attached (three circles,
%                   cfg.Foot's exact locked geometry) -> validatorVehicleCostmap,
%                   which evaluates all three circles per pose, not one
%                   point. Everything below this point (search, endpoint
%                   guards, extraction, cusp/direction logic) is IDENTICAL
%                   regardless of mode - it only ever calls the validator
%                   through the common nav.StateValidator interface
%                   (isStateValid / isMotionValid).

if nargin < 4 || isempty(startPose), startPose = mapInfo.StartPose; end
if nargin < 5 || isempty(goalPose),  goalPose  = mapInfo.GoalPose;  end

tPlanStart = navClock();

%% ---------- state space, bounds derived from the map ----------
ss = stateSpaceSE2;
switch lower(cfg.Foot.Mode)
    case 'single'
        ss.StateBounds = [omap.XWorldLimits; omap.YWorldLimits; -pi pi];
    case 'threecircle'
        % vehicleCostmap exposes MapExtent = [xmin xmax ymin ymax],
        % not XWorldLimits/YWorldLimits (that property belongs to
        % binaryOccupancyMap/occupancyMap, not vehicleCostmap).
        ss.StateBounds = [omap.MapExtent(1,1:2); omap.MapExtent(1,3:4); -pi pi];
    otherwise
        error('planGlobalPath:BadMode', 'cfg.Foot.Mode must be ''single'' or ''threeCircle''.');
end

%% ---------- validator ----------
switch lower(cfg.Foot.Mode)
    case 'single'
        sv = validatorOccupancyMap(ss);
        sv.Map = omap;
    case 'threecircle'
        sv = validatorVehicleCostmap(ss);
        sv.Map = omap;
end
sv.ValidationDistance = cfg.Plan.ValidationDistance_m;

%% ---------- endpoint guards ----------
% Fail loudly here rather than inside the search.
failReason = '';
if ~isStateValid(sv, startPose)
    failReason = 'BadStart';
elseif ~isStateValid(sv, goalPose)
    failReason = 'BadGoal';
end

if ~isempty(failReason)
    planResult = emptyPlanResult(mapInfo, startPose, goalPose, tPlanStart, failReason);
    return;
end

%% ---------- planner ----------
p = plannerHybridAStar(sv, ...
    'MaxNumNodes',      cfg.Plan.MaxNumNodes, ...
    'MaxNumPathStates', cfg.Plan.MaxNumPathStates);
p.MinTurningRadius        = cfg.Plan.MinTurningRadius_m;
p.MotionPrimitiveLength   = cfg.Plan.MotionPrimitiveLength_m;
p.NumMotionPrimitives     = cfg.Plan.NumMotionPrimitives;
p.ForwardCost             = cfg.Plan.ForwardCost;
p.ReverseCost             = cfg.Plan.ReverseCost;
p.DirectionSwitchingCost  = cfg.Plan.DirectionSwitchingCost;
p.AnalyticExpansionInterval = cfg.Plan.AnalyticExpansionInterval;
p.InterpolationDistance   = cfg.Plan.InterpolationDistance_m;
p.MotionDirection         = cfg.Plan.MotionDirection;

%% ---------- search, with automatic fallback ----------
modeUsed = cfg.Plan.SearchMode;
try
    [pathObj, directions, solnInfo] = plan(p, startPose, goalPose, ...
        'SearchMode', cfg.Plan.SearchMode);
catch ME
    if strcmpi(cfg.Plan.SearchMode, 'greedy')
        warning('planGlobalPath:GreedyFailed', ...
            'greedy failed (%s). Retrying exhaustive.', ME.identifier);
        modeUsed = "exhaustive";
        [pathObj, directions, solnInfo] = plan(p, startPose, goalPose, ...
            'SearchMode', 'exhaustive');
    else
        rethrow(ME);
    end
end

tPlanEnd = navClock();

%% ---------- extract ----------
states = pathObj.States;              % N x 3
N = size(states,1);

% Soft-fail for a genuine search failure: start/goal were both valid
% (passed the isStateValid guards above), but plannerHybridAStar still
% couldn't find any route between them (e.g. real inflated obstacles
% genuinely block the corridor). Distinct from the BadStart/BadGoal guard
% above - this covers "valid endpoints, no path exists between them."
% Confirmed via dbstop: this is exactly the N=0, solnInfo.IsPathFound=0
% case, not a malformed-input case - so it gets the same soft-fail
% treatment as Track A rather than being allowed to crash on dGeom(N-1)
% with N<2.
if N < 2
    planResult = emptyPlanResult(mapInfo, startPose, goalPose, tPlanStart, 'SearchFailed');
    return;
end

directions = directions(:);

% Toolbox may return one direction per transition (N-1). Normalise to N.
if numel(directions) == N-1
    directions = [directions; directions(end)];
elseif numel(directions) ~= N
    error('planGlobalPath:DirLength', ...
        'directions length %d matches neither N nor N-1 (N=%d).', numel(directions), N);
end

%% ---------- independent geometric cross-check ----------
% d_k = sign( [cos th_k, sin th_k] . (p_{k+1} - p_k) )
dGeom = ones(N,1);
dxy   = diff(states(:,1:2), 1, 1);
proj  = sum(dxy .* [cos(states(1:N-1,3)), sin(states(1:N-1,3))], 2);
nz    = abs(proj) > 1e-9;
dGeom(1:N-1) = 1;
dGeom(find(nz)) = sign(proj(nz));  %#ok<FNDSB>
dGeom(N) = dGeom(N-1);

disagree = nnz(dGeom ~= directions);

%% ---------- cusps ----------
cuspIdx = find(diff(directions) ~= 0) + 1;

%% ---------- assemble ----------
seg  = vecnorm(diff(states(:,1:2)), 2, 2);

   planResult = struct();
   planResult.States               = states;
   planResult.Directions           = directions;
   planResult.NumWaypoints         = N;
   planResult.CuspIndices          = cuspIdx;
   planResult.NumDirectionSwitches = numel(cuspIdx);
   planResult.PathLength_m         = sum(seg);
   planResult.ReverseLength_m      = sum(seg(directions(1:N-1) < 0));
   planResult.PlanningTime_s       = tPlanEnd - tPlanStart;
   planResult.GenerationTimestamp  = tPlanEnd;
   planResult.MapTimestamp         = mapInfo.Timestamp_s;
   planResult.MapAgeAtPlan_s       = tPlanEnd - mapInfo.Timestamp_s;
   planResult.SearchModeUsed       = modeUsed;
   planResult.DirectionDisagreements = disagree;
   planResult.IsPathFound          = N > 1;
   planResult.SolutionInfo         = solnInfo;
   planResult.StartPose            = startPose;
   planResult.GoalPose             = goalPose;
   planResult.FailureReason          = failReason;
   end
   function planResult = emptyPlanResult(mapInfo, startPose, goalPose, tPlanStart, failReason)
   %EMPTYPLANRESULT  Same field set as a successful plan, zeroed/empty, plus
   %   IsPathFound=false and a reason string. Lets every downstream consumer
   %   (assignVelocityProfile, packPlanBus, your own scripts) read fields
   %   without special-casing a crash - they just see zero waypoints.
   tNow = navClock();
   planResult = struct();
   planResult.States               = zeros(0,3);
   planResult.Directions            = zeros(0,1);
   planResult.NumWaypoints          = 0;
   planResult.CuspIndices           = zeros(0,1);
   planResult.NumDirectionSwitches  = 0;
   planResult.PathLength_m          = 0;
   planResult.ReverseLength_m       = 0;
   planResult.PlanningTime_s        = tNow - tPlanStart;
   planResult.GenerationTimestamp   = tNow;
   planResult.MapTimestamp          = mapInfo.Timestamp_s;
   planResult.MapAgeAtPlan_s        = tNow - mapInfo.Timestamp_s;
   planResult.SearchModeUsed        = "none";
   planResult.DirectionDisagreements = 0;
   planResult.IsPathFound            = false;
   planResult.SolutionInfo           = struct();
   planResult.StartPose              = startPose;
   planResult.GoalPose               = goalPose;
   planResult.FailureReason          = failReason;
   end