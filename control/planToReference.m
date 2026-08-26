function [ref, info] = planToReference(plan, ego, Ts, p, speedCap_mps)
%PLANTOREFERENCE Turn M3's plan bus into a p-by-4 NMPC reference array.
%
%   INPUTS
%     plan  - struct matching M3's delivered bus contract:
%               Waypoints           [5000 x 4]  (x, y, theta, v_signed)
%               NumWaypoints        uint32
%               Directions          [5000 x 1]  (+1 forward, -1 reverse)
%               SeqNum              uint32
%               GenerationTimestamp double (navClock seconds)
%               MapTimestamp        double
%               MapAgeAtPlan_s      double
%     ego   - [X; Y; psi; v] current vehicle state, WORLD frame
%     Ts    - NMPC sample time [s]
%     p     - prediction horizon (number of steps)
%     speedCap_mps - scalar cap from drivingModeParams(mode).speedCap_mps
%
%   OUTPUT
%     ref   - p-by-4, [X Y psi v] at t+Ts ... t+p*Ts
%     info  - diagnostics: nearestIdx, cuspHold, crossTrackErr, numUsed
%
%   THREE THINGS THIS FUNCTION EXISTS TO GET RIGHT:
%
%   1. NumWaypoints gating. Rows beyond NumWaypoints are ZERO PADDING, not
%      waypoints at the origin. Reading numel(Waypoints) gives you 5000 rows
%      of (0,0,0,0), the nearest-point search snaps to the origin, and the
%      vehicle drives at the map origin. Silent, catastrophic, and it looks
%      like an MPC tuning bug.
%
%   2. uint32 -> double conversion happens ONCE, at the top. Leave
%      NumWaypoints as uint32 and any arithmetic on it does integer-saturating
%      maths: idx-1 at idx=0 gives 0, not -1, and interp1 will error strangely.
%
%   3. Cusps. Directions changes sign at a reversal point where M3's profiler
%      has already forced v=0. We must NEVER interpolate a reference across a
%      cusp - that asks the vehicle to teleport through a direction change.
%      The reference clamps at the cusp and waits.

%% ---- 1. Type-normalise and gate --------------------------------------
N = double(plan.NumWaypoints);              % uint32 -> double, exactly once

info = struct('valid', false, 'nearestIdx', 0, 'cuspHold', false, ...
              'crossTrackErr', NaN, 'numUsed', 0, 'reachedEnd', false);

if N < 2
    % Degenerate/absent plan. Hold position, zero speed - fail safe.
    ref = repmat([ego(1), ego(2), ego(3), 0], p, 1);
    return;
end

W = plan.Waypoints(1:N, :);                 % <- the gate. Never plan.Waypoints
D = double(plan.Directions(1:N));

%% ---- 2. Drop duplicate points (zero-length segments break interp1) ---
seg  = hypot(diff(W(:,1)), diff(W(:,2)));
keep = [true; seg > 1e-6];
W = W(keep, :);
D = D(keep);
n = size(W, 1);
if n < 2
    ref = repmat([ego(1), ego(2), ego(3), 0], p, 1);
    return;
end
info.numUsed = n;

%% ---- 3. Arc-length parameterisation ----------------------------------
s = [0; cumsum(hypot(diff(W(:,1)), diff(W(:,2))))];

%% ---- 4. Locate ego on the plan ---------------------------------------
d2 = (W(:,1) - ego(1)).^2 + (W(:,2) - ego(2)).^2;
[dmin2, i0] = min(d2);
info.nearestIdx    = i0;
info.crossTrackErr = sqrt(dmin2);

%% ---- 5. Find the next cusp at or after our position ------------------
sCusp = s(end);                             % default: no cusp, end of path
for k = i0:n-1
    if D(k+1) ~= D(k)
        sCusp = s(k);                       % arc length of the reversal point
        break;
    end
end

%% ---- 6. Unwrap heading before interpolating --------------------------
% Interpolating raw wrapped headings across a +pi/-pi boundary produces a
% reference that spins the long way round. Unwrap first, re-align to ego last.
th = unwrap(W(:,3));

%% ---- 7. Apply the mode speed cap to the profile ----------------------
% Signed: preserves reverse direction, clamps magnitude only.
vProf = sign(W(:,4)) .* min(abs(W(:,4)), abs(speedCap_mps));

%% ---- 8. March forward in TIME along the path -------------------------
% The plan is spaced by distance; the NMPC needs it spaced by time. Integrate
% the profiled speed forward one Ts at a time to get target arc lengths.
sk  = s(i0);
ref = zeros(p, 4);

for k = 1:p
    vk = interp1(s, vProf, sk, 'linear', 'extrap');
    sk = sk + abs(vk) * Ts;

    if sk >= sCusp - 1e-6
        sk = sCusp;                         % clamp - never step past a cusp
        info.cuspHold = true;
    end
    if sk >= s(end) - 1e-6
        sk = s(end);
        info.reachedEnd = true;
    end

    ref(k,1) = interp1(s, W(:,1), sk);
    ref(k,2) = interp1(s, W(:,2), sk);
    ref(k,3) = interp1(s, th,     sk);
    ref(k,4) = interp1(s, vProf,  sk);
end

%% ---- 9. Re-align reference heading to ego's branch -------------------
% Ego psi is a continuous, unwrapped quantity that can be anywhere on the
% real line. The reference must live on the same 2*pi branch or the heading
% error term explodes and the controller commands a full spin.
ref(:,3) = unwrap(ref(:,3));
ref(:,3) = ref(:,3) + round((ego(3) - ref(1,3)) / (2*pi)) * 2*pi;

info.valid = true;
end
