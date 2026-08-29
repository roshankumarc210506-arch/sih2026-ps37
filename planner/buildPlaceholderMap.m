function [omap, rawmap, mapInfo] = buildPlaceholderMap(cfg)
%BUILDPLACEHOLDERMAP  Per-class obstacle map with per-class inflation.
%
%   Each obstacle class is rasterised into its OWN map, inflated ONCE by
%   (ego footprint + class risk margin), then combined by logical OR.
%
%   Never call inflate() twice on the same map: the second call inflates
%   the already-inflated cells and the radii compound instead of maxing.

W   = cfg.Map.Width_m;
H   = cfg.Map.Height_m;
res = cfg.Map.Resolution_cpm;

% Ego disc radius depends on footprint mode
switch lower(cfg.Foot.Mode)
    case 'single'
        egoR = cfg.Foot.Radius_m;
    case 'threecircle'
        egoR = cfg.Foot.CircleRadius_m;
    otherwise
        error('buildPlaceholderMap:BadMode', ...
              'cfg.Foot.Mode must be ''single'' or ''threeCircle''.');
end

%% ---------- world/grid lookup, computed once ----------
ref    = binaryOccupancyMap(W, H, res);
nRows  = ref.GridSize(1);
nCols  = ref.GridSize(2);
[rr, cc] = ndgrid(1:nRows, 1:nCols);
ij = [rr(:), cc(:)];
xy = grid2world(ref, ij);          % row 1 = max y (top), consistent on reshape

%% ---------- obstacle definitions ----------
% rect = [xmin xmax ymin ymax] in world metres
obs = struct('class', {}, 'rect', {}, 'label', {});

obs(end+1) = struct('class','static',    'rect',[ 0 60,  0    1.5 ], 'label','south kerb');
obs(end+1) = struct('class','static',    'rect',[ 0 60, 38.5 40   ], 'label','north kerb');
obs(end+1) = struct('class','static',    'rect',[24 28,  0   12   ], 'label','compound wall');

obs(end+1) = struct('class','vehicle',   'rect',[ 8 12, 30   38.5 ], 'label','parked row (L)');
obs(end+1) = struct('class','vehicle',   'rect',[46 50, 27   32   ], 'label','parked cluster (R)');

obs(end+1) = struct('class','pedestrian','rect',[46 46.6, 8  8.6  ], 'label','pedestrian');

obs(end+1) = struct('class','animal',    'rect',[37 38.5, 16 17.5 ], 'label','cattle');

%% ---------- per-class rasterise, inflate, OR ----------
classes  = unique({obs.class});
combined = false(nRows, nCols);
rawOcc   = false(nRows, nCols);
layers   = struct('class',{},'riskMargin_m',{},'totalInflation_m',{},'freeFrac',{});

for k = 1:numel(classes)
    cls  = classes{k};
    risk = cfg.Risk.(sprintf('%s_m', cls));
    rTot = egoR + risk;

    sel  = strcmp({obs.class}, cls);
    mask = false(size(xy,1), 1);
    rects = {obs(sel).rect};
    for r = 1:numel(rects)
        R = rects{r};
        mask = mask | (xy(:,1) >= R(1) & xy(:,1) <= R(2) & ...
                       xy(:,2) >= R(3) & xy(:,2) <= R(4));
    end

    layerMat = reshape(mask, nRows, nCols);
    rawOcc   = rawOcc | layerMat;

    layer = binaryOccupancyMap(layerMat, res);
    inflate(layer, rTot);                       % ONCE, on this class only
    infMat = occupancyMatrix(layer) > 0;

    combined = combined | infMat;

    layers(end+1) = struct( ...
        'class', cls, 'riskMargin_m', risk, ...
        'totalInflation_m', rTot, ...
        'freeFrac', 1 - nnz(infMat)/numel(infMat));  %#ok<AGROW>
end

%% ---------- assemble ----------
omap   = binaryOccupancyMap(combined, res);
rawmap = binaryOccupancyMap(rawOcc,   res);

mapInfo = struct();
mapInfo.Timestamp_s    = navClock();
mapInfo.Resolution_cpm = res;
mapInfo.CellSize_m     = 1/res;
mapInfo.EgoRadius_m    = egoR;
mapInfo.FootprintMode  = cfg.Foot.Mode;
mapInfo.Layers         = layers;
mapInfo.Obstacles      = obs;
mapInfo.FreeFraction   = 1 - nnz(combined)/numel(combined);

% Scenario endpoints travel with the map so Step 4 can't drift out of sync
mapInfo.StartPose = [ 4, 22, 0];
mapInfo.GoalPose  = [56, 12, 0];
end