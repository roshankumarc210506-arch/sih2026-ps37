function pt = findOnRoadPoint(roadBoundaryCells, xQuery, yRange)
%FINDONROADPOINT  Sample yRange at a given x, return the first point
%   found inside the road polygon. Diagnostic only - for picking a
%   sensible start/goal when the old placeholder defaults don't apply.
%
%   REVERTED (Open Items #3, attempted then rolled back): tried
%   estimating heading from the local road boundary tangent instead of
%   hardcoding 0, to fix a spurious start-of-path cusp (Session Context
%   3f). CONFIRMED VIA TESTING that the tangent estimate breaks planning
%   entirely on the real village-road scenario - a ~15.6 deg heading
%   offset from the tangent calc was enough to make
%   plannerHybridAStar expand zero nodes (NumNodes=0) from the start
%   pose, on a corridor narrow enough relative to the 2.52m ego
%   footprint disc that there's little tolerance for heading error.
%   A/B-tested: identical inputs, heading=0 -> IsPathFound=1;
%   tangent-derived heading -> IsPathFound=0.
%
%   The tangent code (localBoundaryHeading, below) is left in place but
%   UNUSED, for whoever picks this back up: root cause suspected but not
%   confirmed to be the boundary loop's end-cap regions (where the
%   polygon transitions between the two road edges) producing a
%   perpendicular-to-road tangent rather than an along-road one - a
%   boundary plot near x=4 showed the nearest-vertex window sitting very
%   close to the loop's start. Needs either (a) excluding cap-adjacent
%   vertices when picking the nearest one, or (b) a real centerline
%   extraction instead of using the raw edge-loop tangent. Doc flags
%   this as low-priority/diagnostic-only, so reverting to the known-good
%   heading=0 rather than blocking on a proper fix.
b = roadBoundaryCells{1};   % single-boundary scenario, confirmed earlier
for y = yRange
    if inpolygon(xQuery, y, b(:,1), b(:,2))
        pt = [xQuery, y, 0];
        return;
    end
end
error('findOnRoadPoint:NotFound', 'No on-road point at x=%.1f in given y range.', xQuery);
end

function h = localBoundaryHeading(b, pt) %#ok<DEFNU>
%LOCALBOUNDARYHEADING  UNUSED (see revert note above). Coarse
%   tangent-direction estimate at the boundary vertex nearest pt, via
%   central difference over neighboring vertices. Kept for whoever picks
%   this back up - not currently called.
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