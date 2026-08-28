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
%   AND every agent's worst-case inflated disc, each plus a margin.
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
 
if nargin < 6 || isempty(cfg), cfg = planner_config(); end
if nargin < 7 || isempty(startPose)
    error('buildRealOccupancyMap:NoStart', 'startPose is required - no default for real scenarios. Use findOnRoadPoint.m.');
end
if nargin < 8 || isempty(goalPose)
    error('buildRealOccupancyMap:NoGoal', 'goalPose is required - no default for real scenarios. Use findOnRoadPoint.m.');
end
 
res = cfg.Map.Resolution_cpm;
 
switch lower(cfg.Foot.Mode)
    case 'single',      egoR = cfg.Foot.Radius_m;
    case 'threecircle', egoR = cfg.Foot.CircleRadius_m;
    otherwise, error('buildRealOccupancyMap:BadMode', 'cfg.Foot.Mode must be ''single'' or ''threeCircle''.');
end
 
%% ---------- PASS 1: resolve every agent's world position + obstacle radius ----------
predIds = [predictions.id];
agentInfo = struct('id',{},'class',{},'posWorld',{},'riskMargin_m',{},'extent_m',{},'obstacleRadius_m',{});
 
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
 
    agentInfo(end+1) = struct('id', tr.id, 'class', char(tr.class), ...
        'posWorld', posWorld, 'riskMargin_m', riskMargin, ...
        'extent_m', extent, 'obstacleRadius_m', obstacleRadius); %#ok<AGROW>
end
 
%% ---------- bounds: road geometry UNION every agent's worst-case disc ----------
roadMargin_m = 5;
b = computeMapBoundsFromRoad(roadBoundaryCells, roadMargin_m);
 
for k = 1:numel(agentInfo)
    a = agentInfo(k);
    b.xmin = min(b.xmin, a.posWorld(1) - a.obstacleRadius_m);
    b.xmax = max(b.xmax, a.posWorld(1) + a.obstacleRadius_m);
    b.ymin = min(b.ymin, a.posWorld(2) - a.obstacleRadius_m);
    b.ymax = max(b.ymax, a.posWorld(2) + a.obstacleRadius_m);
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
[staticOccupied, staticRaw] = buildStaticLayerFromRoad(roadBoundaryCells, ref, cfg);
 
%% ---------- PASS 2: rasterize each agent onto the SAME grid ----------
combined = staticOccupied;
rawOcc   = staticRaw;
layers   = struct('id',{},'class',{},'riskMargin_m',{},'obstacleRadius_m',{});
 
for k = 1:numel(agentInfo)
    a = agentInfo(k);
    d = hypot(xy(:,1) - a.posWorld(1), xy(:,2) - a.posWorld(2));
    combined = combined | reshape(d <= a.obstacleRadius_m, nRows, nCols);
    rawOcc   = rawOcc   | reshape(d <= a.extent_m,          nRows, nCols);
    layers(end+1) = struct('id', a.id, 'class', a.class, ...
        'riskMargin_m', a.riskMargin_m, 'obstacleRadius_m', a.obstacleRadius_m); %#ok<AGROW>
end
 
omap = binaryOccupancyMap(combined, res);
omap.LocalOriginInWorld = [b.xmin, b.ymin];
rawmap = binaryOccupancyMap(rawOcc, res);
rawmap.LocalOriginInWorld = [b.xmin, b.ymin];
 
%% ---------- assemble ----------
mapInfo = struct();
mapInfo.Timestamp_s    = navClock();
mapInfo.Resolution_cpm = res;
mapInfo.CellSize_m     = 1/res;
mapInfo.EgoRadius_m    = egoR;
mapInfo.FootprintMode  = cfg.Foot.Mode;
mapInfo.Layers         = layers;
mapInfo.NumTracksUsed  = numTracks;
mapInfo.EgoPose        = egoPose;
mapInfo.FreeFraction   = 1 - nnz(combined)/numel(combined);
mapInfo.Bounds         = b;
mapInfo.StartPose      = startPose;
mapInfo.GoalPose       = goalPose;
end
 
