function [staticOccupied, staticRaw] = buildStaticLayerFromRoad(roadBoundaryCells, refMap, cfg)
%BUILDSTATICLAYERFROMROAD  Rasterize M5's road() geometry into a static
%   occupancy layer: on-road = free, off-road = occupied.
%
%   roadBoundaryCells : output of roadBoundaries(scenario) - cell array,
%                        one cell per road, each an Nx3 closed-loop
%                        polygon [x y z] in WORLD coordinates.
%   refMap             : a binaryOccupancyMap defining the grid to
%                        rasterize onto - same size/resolution/world
%                        extent as whatever you'll OR this into.
%   cfg                 : needs cfg.Risk.static_m
%
%   staticOccupied : inflated static layer (off-road + risk margin).
%   staticRaw      : UNINFLATED off-road mask - exact road edge, no
%                    margin. Feeds checkPathFootprint.m's ground-truth
%                    audit, same convention buildPlaceholderMap.m used
%                    for its hardcoded kerb rectangles.
%
%   Multiple roads are combined by OR-ing each road's own on-road mask,
%   NOT by polyshape union - avoids self-intersection issues at
%   junctions where two road() calls overlap.

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

offRoad = ~onRoad;                  % off-road = occupied
staticRaw = offRoad;                % exact edge, no margin - ground truth

layer = binaryOccupancyMap(offRoad, refMap.Resolution);
if cfg.Risk.static_m > 0
    inflate(layer, cfg.Risk.static_m);
end
staticOccupied = occupancyMatrix(layer) > 0;
end