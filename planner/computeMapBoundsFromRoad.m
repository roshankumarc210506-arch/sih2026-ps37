function bounds = computeMapBoundsFromRoad(roadBoundaryCells, margin_m)
%COMPUTEMAPBOUNDSFROMROAD  World-frame bounding box covering all road
%   boundary cells, padded by margin_m so inflated obstacles near the
%   edge don't get clipped by the map itself.
allXY = vertcat(roadBoundaryCells{:});
bounds.xmin = min(allXY(:,1)) - margin_m;
bounds.xmax = max(allXY(:,1)) + margin_m;
bounds.ymin = min(allXY(:,2)) - margin_m;
bounds.ymax = max(allXY(:,2)) + margin_m;
bounds.Width_m  = bounds.xmax - bounds.xmin;
bounds.Height_m = bounds.ymax - bounds.ymin;
end