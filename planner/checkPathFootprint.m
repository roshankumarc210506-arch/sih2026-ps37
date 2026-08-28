function audit = checkPathFootprint(planResult, rawmap, cfg)
%CHECKPATHFOOTPRINT  Post-hoc clearance audit against the RAW (uninflated)
%   obstacle map. Answers: given what the planner actually used (a single
%   2.52 m disc, or three circles, per cfg.Foot.Mode), how much real
%   clearance does the true 4.7 x 1.8 m body have at its closest point?
%
%   This does NOT change planning behavior. It is a report.

S = planResult.States;
N = size(S,1);

freeMask = ~(occupancyMatrix(rawmap) > 0);
distToObs_m = bwdist(~freeMask) / rawmap.Resolution;   % m, requires Image Processing Toolbox

switch lower(cfg.Foot.Mode)
    case 'single'
        offsets = 0;                         % single point: the state itself
        radii   = cfg.Foot.Radius_m;
    case 'threecircle'
        offsets = cfg.Foot.CircleOffsets_m;
        radii   = cfg.Foot.CircleRadius_m * ones(size(offsets));
    otherwise
        error('checkPathFootprint:BadMode', 'Unknown cfg.Foot.Mode.');
end

nC = numel(offsets);
clearance_m = inf(N, nC);   % clearance = distance-to-nearest-obstacle MINUS this circle's radius

for c = 1:nC
    cx = S(:,1) + offsets(c).*cos(S(:,3));
    cy = S(:,2) + offsets(c).*sin(S(:,3));
    ij = world2grid(rawmap, [cx, cy]);
    ij(:,1) = min(max(ij(:,1),1), rawmap.GridSize(1));
    ij(:,2) = min(max(ij(:,2),1), rawmap.GridSize(2));
    lin = sub2ind(rawmap.GridSize, ij(:,1), ij(:,2));
    clearance_m(:,c) = distToObs_m(lin) - radii(c);
end

worstClearance_m = min(clearance_m, [], 2);
[minClear, worstIdx] = min(worstClearance_m);

% Also compute what the TRUE body extremes need, regardless of mode,
% so the report always shows the honest number.
frontPt = S(:,1:2) + cfg.Veh.FrontAxleToBumper_m .* [cos(S(:,3)), sin(S(:,3))];
rearPt  = S(:,1:2) - cfg.Veh.RearOverhang_m       .* [cos(S(:,3)), sin(S(:,3))];
halfW   = cfg.Veh.Width_m/2;
% four true body corners per waypoint, checked against raw clearance directly
corners_m = inf(N,4);
cornerDefs = [ 1  1; 1 -1; -1  1; -1 -1];  % [front/rear sign, left/right sign]
for k = 1:4
    base = (cornerDefs(k,1)>0) .* frontPt + (cornerDefs(k,1)<0) .* rearPt;
    perp = cornerDefs(k,2)*halfW .* [-sin(S(:,3)), cos(S(:,3))];
    pt = base + perp;
    ij = world2grid(rawmap, pt);
    ij(:,1) = min(max(ij(:,1),1), rawmap.GridSize(1));
    ij(:,2) = min(max(ij(:,2),1), rawmap.GridSize(2));
    lin = sub2ind(rawmap.GridSize, ij(:,1), ij(:,2));
    corners_m(:,k) = distToObs_m(lin);   % raw clearance, radius 0 - these are exact body points
end
trueBodyClearance_m = min(corners_m, [], 2);
[trueMin, trueWorstIdx] = min(trueBodyClearance_m);

audit = struct();
audit.FootprintMode           = cfg.Foot.Mode;
audit.PerWaypointClearance_m  = worstClearance_m;
audit.MinClearance_m          = minClear;
audit.MinClearanceWaypointIdx = worstIdx;
audit.MinClearancePose        = S(worstIdx,:);
audit.NumWaypointsNegative    = nnz(worstClearance_m < 0);
audit.TrueBodyClearance_m     = trueBodyClearance_m;
audit.TrueBodyMinClearance_m  = trueMin;
audit.TrueBodyWorstIdx        = trueWorstIdx;
audit.TrueBodyNumCollisions   = nnz(trueBodyClearance_m < 0);
end