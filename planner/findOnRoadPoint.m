function pt = findOnRoadPoint(roadBoundaryCells, xQuery, yRange)
%FINDONROADPOINT  Sample yRange at a given x, return a point centered in
%   the on-road band there. Diagnostic only - for picking a sensible
%   start/goal when the old placeholder defaults don't apply.
%
%   FIXED (Day 4, confirmed via testAllScenarios.m + direct measurement -
%   checkEdgeHugging.m): previously returned the FIRST y satisfying
%   inpolygon scanning from min(yRange) upward, which lands right at
%   (within ~0.002m of) the near road edge regardless of how much usable
%   width actually exists. Confirmed this caused persistent BadStart on
%   Cattle Crossing and Highway Merge despite those roads having 2m+ MORE
%   clearance available than the vehicle's 2.52m footprint disc needs -
%   never used, because the returned point sat almost exactly on the
%   edge instead of using that available width. This ALSO explains the
%   suspiciously-exact TrueBodyMinClearance_m=0.000 seen earlier on the
%   real village-road test (same bug, just narrowly surviving there
%   instead of failing outright).
%
%   Now finds the full on-road y-extent at xQuery (via fine internal
%   sampling, independent of the caller's yRange step size) and returns
%   its CENTER. Heading is still hardcoded to 0 (separate revert,
%   documented below/earlier - unrelated to this fix).
%
%   ASSUMES a single contiguous on-road band at any given x (true for
%   all 5 of M5's current scenarios, checked directly). A future
%   scenario with a median/divider creating two separate on-road bands
%   at the same x would break this assumption (would incorrectly average
%   across the gap, landing on the divider) - not currently an issue,
%   flagged for whoever adds such a scenario.
b = roadBoundaryCells{1};   % single-boundary scenario, confirmed earlier

yLo = min(yRange); yHi = max(yRange);
if yHi <= yLo
    error('findOnRoadPoint:BadYRange', 'yRange must span a nonzero range.');
end

nFine = max(2000, round((yHi - yLo) / 0.05));
yFine = linspace(yLo, yHi, nFine);
onRoad = false(size(yFine));
for i = 1:numel(yFine)
    onRoad(i) = inpolygon(xQuery, yFine(i), b(:,1), b(:,2));
end

if ~any(onRoad)
    error('findOnRoadPoint:NotFound', 'No on-road point at x=%.1f in given y range.', xQuery);
end

yOnRoad = yFine(onRoad);
yCenter = (min(yOnRoad) + max(yOnRoad)) / 2;
pt = [xQuery, yCenter, 0];
end

function h = localBoundaryHeading(b, pt) %#ok<DEFNU>
%LOCALBOUNDARYHEADING  UNUSED (see separate revert note - tangent-based
%   heading broke real pathfinding, reverted to heading=0). Kept for
%   whoever picks this back up - not currently called. Unrelated to the
%   centering fix above.
n = size(b, 1);
if all(b(1, 1:2) == b(end, 1:2))
    n = n - 1;
end
d = hypot(b(1:n,1) - pt(1), b(1:n,2) - pt(2));
[~, idx] = min(d);
iPrev = mod(idx - 2, n) + 1;
iNext = mod(idx,     n) + 1;
tangent = b(iNext, 1:2) - b(iPrev, 1:2);
h = atan2(tangent(2), tangent(1));
end