   function planResult = planGlobalPath(omap, cfg, mapInfo, startPose, goalPose)
%PLANGLOBALPATH  Hybrid A* over SE(2) with Reeds-Shepp analytic expansion.
%
%   Returns geometry only: [x y theta] + directions. No velocity, no
%   per-waypoint timestamps - those are the profiler's job (Step 5).

if nargin < 4 || isempty(startPose), startPose = mapInfo.StartPose; end
if nargin < 5 || isempty(goalPose),  goalPose  = mapInfo.GoalPose;  end

tPlanStart = navClock();

%% ---------- state space, bounds derived from the map ----------
ss = stateSpaceSE2;
ss.StateBounds = [omap.XWorldLimits; omap.YWorldLimits; -pi pi];

%% ---------- validator ----------
sv = validatorOccupancyMap(ss);
sv.Map = omap;
sv.ValidationDistance = cfg.Plan.ValidationDistance_m;

%% ---------- endpoint guards ----------
% Fail loudly here rather than inside the search.
if ~isStateValid(sv, startPose)
    error('planGlobalPath:BadStart', ...
        'Start [%.2f %.2f %.2f] is invalid (occupied or out of bounds).', startPose);
end
if ~isStateValid(sv, goalPose)
    error('planGlobalPath:BadGoal', ...
        'Goal [%.2f %.2f %.2f] is invalid (occupied or out of bounds).', goalPose);
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
   end
