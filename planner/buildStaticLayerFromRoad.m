function [staticOccupied, staticRaw] = buildStaticLayerFromRoad(roadBoundaryCells, refMap, cfg, egoR)
%BUILDSTATICLAYERFROMROAD  Rasterize M5's road() geometry into a static
%   occupancy layer: on-road = free, off-road = occupied.
%
%   egoR : the SAME ego footprint radius used against agent obstacles
%          (cfg.Foot.Radius_m or cfg.Foot.CircleRadius_m, depending on
%          cfg.Foot.Mode). REQUIRED - without it, road edges are only
%          inflated by cfg.Risk.static_m (currently 0), meaning the
%          planner treats the vehicle as a POINT against road edges
%          while treating it as a proper disc against agents. Confirmed
%          this WAS the actual live behavior before this fix.
%
%   [... rest of header unchanged ...]

nRows = refMap.GridSize(1);
nCols = refMap.GridSize(2);
[rr, cc] = ndgrid(1:nRows, 1:nCols);
xy = grid2world(refMap, [rr(:), cc(:)]);

onRoad = false(nRows, nCols);

for k = 1:numel(roadBoundaryCells)
    b = roadBoundaryCells{k};
    if ~isequal(b(1,1:2), b(end,1:2))
        warning('buildStaticLayerFromRoad:NotClosed', ...
            'Boundary cell %d is not a closed loop - skipping.', k);
        continue;
    end
    in = inpolygon(xy(:,1), xy(:,2), b(:,1), b(:,2));
    onRoad = onRoad | reshape(in, nRows, nCols);
end

offRoad = ~onRoad;
staticRaw = offRoad;

layer = binaryOccupancyMap(offRoad, refMap.Resolution);
totalInflation = egoR + cfg.Risk.static_m;
% inflate() requires a strictly positive radius - errors on exactly 0.
% threeCircle mode passes egoR=0 (footprint applied per-pose downstream
% by validatorVehicleCostmap instead), and cfg.Risk.static_m defaults to
% 0.00 - so this sum can legitimately be 0. Skip inflate() in that case;
% skipping it is correct, not a workaround, since inflating by 0 would be
% a no-op anyway.
if totalInflation > 0
    inflate(layer, totalInflation);
end
staticOccupied = occupancyMatrix(layer) > 0;
end