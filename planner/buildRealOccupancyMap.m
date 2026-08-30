function [omap, rawmap, mapInfo] = buildRealOccupancyMap(tracks, numTracks, predictions, egoPose, roadBoundaryCells, cfg, startPose, goalPose)
%BUILDREALOCCUPANCYMAP  Per-class inflated occupancy grid from real
%   perception+prediction data AND real road geometry (M1-confirmed:
%   roadBoundaries(scenario), NOT the tracker - targetPoses/targetOutlines
%   are separate paths).
%
%   SIGNATURE CHANGED from v1: roadBoundaryCells is now a required arg,
%   inserted before cfg.
%
%   Map bounds are DERIVED, not fixed: sized to cover the road geometry
%   AND every agent's worst-case swept region, each plus a margin.
%   cfg.Map.Width_m/Height_m are no longer used to size the grid - only
%   cfg.Map.Resolution_cpm is.
%
%   startPose/goalPose are now REQUIRED, no silent default. Old
%   cfg.Map.DefaultStart/DefaultGoal were tuned for the placeholder
%   scenario and are off this road entirely - use findOnRoadPoint.m to
%   get real ones instead of guessing.
%
%   DESIGN CHOICE (still flagged, unchanged from v1): off-road is fully
%   occupied - the planner cannot leave the road corridor at all. Worth
%   revisiting with the team once M4/M5 have opinions on shoulder use.
%
%   CHANGED (Open Items #2): the flat agentHalfExtent_m = 0.3 placeholder
%   is replaced by classToAgentExtent(tr.class), M1's confirmed-sound
%   diagonal-inflation stopgap (Session Context 3i). This makes the risk
%   radius per-class instead of one-size-fits-all, and also drives the
%   raw (unrisked) audit mask, which previously stayed wrong at a flat
%   0.3m regardless of agent size.
%
%   MODE BRANCH (new - three-circle collision checking):
%   'single'      : UNCHANGED behavior. egoR = cfg.Foot.Radius_m is baked
%                   directly into obstacle AND road-edge inflation
%                   (Minkowski-sum trick), map returned as a plain
%                   binaryOccupancyMap, planGlobalPath.m does a single
%                   point check per state via validatorOccupancyMap.
%   'threeCircle' : Baking a single scalar egoR into the grid would only
%                   ever check ONE point (the rear axle) against ONE
%                   circle radius - it would silently ignore the other
%                   two circles offset forward along the body, leaving
%                   the vehicle's mid/front section completely unchecked
%                   during planning. So for this mode, NO ego radius is
%                   baked into the grid at all (obstacleRadius_m below
%                   omits egoR entirely, and buildStaticLayerFromRoad is
%                   called with egoR=0). Instead this function returns a
%                   vehicleCostmap with a real InflationCollisionChecker
%                   attached (buildThreeCircleCollisionChecker.m,
%                   reproducing cfg.Foot's exact locked circle geometry) -
%                   the ego footprint is applied per-pose by
%                   validatorVehicleCostmap in planGlobalPath.m, which
%                   genuinely evaluates all three circles, not one point.
%                   Requires Automated Driving Toolbox (confirmed
%                   licensed for this project).

if nargin < 6 || isempty(cfg), cfg = planner_config(); end
if nargin < 7 || isempty(startPose)
    error('buildRealOccupancyMap:NoStart', 'startPose is required - no default for real scenarios. Use findOnRoadPoint.m.');
end
if nargin < 8 || isempty(goalPose)
    error('buildRealOccupancyMap:NoGoal', 'goalPose is required - no default for real scenarios. Use findOnRoadPoint.m.');
end

res = cfg.Map.Resolution_cpm;

switch lower(cfg.Foot.Mode)
    case 'single'
        egoR              = cfg.Foot.Radius_m;   % baked into the grid, as before
        staticLayerEgoR   = egoR;
        boundsMarginR     = egoR;
    case 'threecircle'
        egoR              = 0;                    % NOT baked into obstacle inflation
        staticLayerEgoR   = 0;                     % NOT baked into road-edge inflation
        % Bounds sizing only (does not affect collision correctness):
        % how far any point on the swept vehicle body can be from the
        % state's (x,y) rear-axle point, so map bounds don't clip a real
        % corner case near the map edge.
        boundsMarginR     = cfg.Foot.CircleRadius_m + max(abs(cfg.Foot.CircleOffsets_m));
    otherwise
        error('buildRealOccupancyMap:BadMode', 'cfg.Foot.Mode must be ''single'' or ''threeCircle''.');
end

%% ---------- PASS 1: resolve every agent's world position + obstacle radius ----------
% CHANGED (Simulink-safety pass): agentInfo is now preallocated to a
% FIXED size (numel(tracks) - tracks is already MaxTracks-sized coming
% from M1's contract) instead of growing via (end+1), which cannot run
% inside a MATLAB Function block. numAgents tracks how many of the
% preallocated slots are actually filled (<= numTracks, since some
% tracks may be ~valid and skipped).
predIds = [predictions.id];
maxAgents = numel(tracks);
agentInfo = repmat(struct('id',0,'class','','posWorld',[0 0],'riskMargin_m',0,'extent_m',0,'obstacleRadius_m',0), maxAgents, 1);
numAgents = 0;

for k = 1:numTracks
    tr = tracks(k);
    if ~tr.valid, continue; end

    posWorld = transformEgoToWorld([tr.x, tr.y], egoPose);
    pIdx = find(predIds == tr.id, 1);
    if isempty(pIdx)
        warning('buildRealOccupancyMap:NoPrediction', ...
            'No prediction for track id %d - using 0 uncertainty growth.', tr.id);
        maxUncertainty = 0;
    else
        maxUncertainty = max(predictions(pIdx).uncertainty_radius);
    end

    riskField  = classToRiskField(tr.class);
    riskMargin = cfg.Risk.(riskField);
    extent     = classToAgentExtent(tr.class);
    obstacleRadius = egoR + riskMargin + extent + maxUncertainty;

    numAgents = numAgents + 1;
    agentInfo(numAgents) = struct('id', tr.id, 'class', char(tr.class), ...
        'posWorld', posWorld, 'riskMargin_m', riskMargin, ...
        'extent_m', extent, 'obstacleRadius_m', obstacleRadius);
end

%% ---------- bounds: road geometry UNION every agent's worst-case swept region ----------
roadMargin_m = 5;
b = computeMapBoundsFromRoad(roadBoundaryCells, roadMargin_m);

for k = 1:numAgents
    a = agentInfo(k);
    reach = a.obstacleRadius_m + boundsMarginR;   % obstacle risk radius + how far ego sticks out
    b.xmin = min(b.xmin, a.posWorld(1) - reach);
    b.xmax = max(b.xmax, a.posWorld(1) + reach);
    b.ymin = min(b.ymin, a.posWorld(2) - reach);
    b.ymax = max(b.ymax, a.posWorld(2) + reach);
end
b.Width_m  = b.xmax - b.xmin;
b.Height_m = b.ymax - b.ymin;

%% ---------- reference grid at the correct origin ----------
ref = binaryOccupancyMap(b.Width_m, b.Height_m, res);
ref.LocalOriginInWorld = [b.xmin, b.ymin];

nRows = ref.GridSize(1); nCols = ref.GridSize(2);
[rr, cc] = ndgrid(1:nRows, 1:nCols);
xy = grid2world(ref, [rr(:), cc(:)]);

%% ---------- static layer: off-road = occupied ----------
[staticOccupied, staticRaw] = buildStaticLayerFromRoad(roadBoundaryCells, ref, cfg, staticLayerEgoR);

%% ---------- PASS 2: rasterize each agent onto the SAME grid ----------
combined = staticOccupied;
rawOcc   = staticRaw;
% CHANGED (Simulink-safety pass): fixed-size, same reasoning as agentInfo
% above. numAgents is reused as the fill count since every agentInfo
% entry gets exactly one layers entry, one-to-one, in this loop.
layers = repmat(struct('id',0,'class','','riskMargin_m',0,'obstacleRadius_m',0), maxAgents, 1);

for k = 1:numAgents
    a = agentInfo(k);
    d = hypot(xy(:,1) - a.posWorld(1), xy(:,2) - a.posWorld(2));
    combined = combined | reshape(d <= a.obstacleRadius_m, nRows, nCols);
    rawOcc   = rawOcc   | reshape(d <= a.extent_m,          nRows, nCols);
    layers(k) = struct('id', a.id, 'class', a.class, ...
        'riskMargin_m', a.riskMargin_m, 'obstacleRadius_m', a.obstacleRadius_m);
end

combinedMap = binaryOccupancyMap(combined, res);
combinedMap.LocalOriginInWorld = [b.xmin, b.ymin];

switch lower(cfg.Foot.Mode)
    case 'single'
        omap = combinedMap;   % unchanged: plain binaryOccupancyMap, egoR already baked in above
    case 'threecircle'
        ccConfig = buildThreeCircleCollisionChecker(cfg);
        omap = vehicleCostmap(combinedMap, 'CollisionChecker', ccConfig);
end

% rawmap is mode-independent - always the raw (unrisked, no ego at all)
% agent+road mask, used only by checkPathFootprint.m's post-hoc audit.
rawmap = binaryOccupancyMap(rawOcc, res);
rawmap.LocalOriginInWorld = [b.xmin, b.ymin];

%% ---------- assemble ----------
mapInfo = struct();
mapInfo.Timestamp_s    = navClock();
mapInfo.Resolution_cpm = res;
mapInfo.CellSize_m     = 1/res;
mapInfo.FootprintMode  = cfg.Foot.Mode;
switch lower(cfg.Foot.Mode)
    case 'single'
        mapInfo.EgoRadius_m = egoR;   % single scalar, baked into the grid
    case 'threecircle'
        % NOT a single effective radius - three circles, applied per-pose
        % by validatorVehicleCostmap, not baked into this grid at all.
        mapInfo.EgoRadius_m     = cfg.Foot.CircleRadius_m;   % per-circle radius, for reference
        mapInfo.CircleOffsets_m = cfg.Foot.CircleOffsets_m;
end
mapInfo.Layers         = layers;      % fixed-size (maxAgents); see NumLayers for real count
mapInfo.NumLayers      = numAgents;
mapInfo.NumTracksUsed  = numTracks;
mapInfo.EgoPose        = egoPose;
mapInfo.FreeFraction   = 1 - nnz(combined)/numel(combined);
mapInfo.Bounds         = b;
mapInfo.StartPose      = startPose;
mapInfo.GoalPose       = goalPose;
end