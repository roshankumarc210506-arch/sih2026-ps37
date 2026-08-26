function plan = makeDummyPlanBus(seqNum, nowSec, withCusp, pathLen_m)
%MAKEDUMMYPLANBUS Synthesise a plan bus in M3's EXACT delivered format.
%
%   Lets you build and test the whole control path before M3's real planner
%   lands. Reproduces the awkward parts of the real bus deliberately: fixed
%   5000-row arrays with zero padding, uint32 counters, a signed velocity
%   column, and optionally a direction-change cusp.
%
%   plan = makeDummyPlanBus(seqNum, nowSec, withCusp, pathLen_m)
%
%   pathLen_m defaults to 120 m. The v1 default of 60 m was too short:
%   the vehicle reached the end of the plan at t~13 s and sat parked for
%   the rest of the run, so the YIELD and STOP modes were never exercised
%   on a moving vehicle. Keep the path longer than the run can consume.

if nargin < 3 || isempty(withCusp),  withCusp  = false; end
if nargin < 4 || isempty(pathLen_m), pathLen_m = 120;   end

MAXWP = 5000;                               % locked array height

%% ---- Forward S-curve --------------------------------------------------
ds  = 0.5;                                  % waypoint spacing [m]
xF  = (0:ds:pathLen_m)';
yF  = 3*sin(2*pi*xF/40);
thF = atan2(3*(2*pi/40)*cos(2*pi*xF/40), 1);
dF  = ones(numel(xF), 1);

%% ---- Optional reverse segment (exercises cusp handling) --------------
if withCusp
    sR  = (ds:ds:8)';
    xR  = xF(end) - sR*cos(thF(end));       % reverse along current heading
    yR  = yF(end) - sR*sin(thF(end));
    thR = repmat(thF(end), numel(sR), 1);   % heading unchanged in reverse
    dR  = -ones(numel(sR), 1);

    x = [xF; xR];  y = [yF; yR];  th = [thF; thR];  d = [dF; dR];
else
    x = xF;  y = yF;  th = thF;  d = dF;
end

n = numel(x);
assert(n <= MAXWP, 'Plan has %d waypoints, bus holds %d.', n, MAXWP);

%% ---- Velocity profile (stands in for M3's assignVelocityProfile) -----
vp    = vehicleParams();
vNom  = 8.33;                               % CRUISE nominal
kappa = localCurvature(x, y);
vCurv = sqrt(vp.aLatMax ./ max(abs(kappa), 1e-6));   % lateral-accel cap
v     = min(vNom, vCurv);
v     = v .* d;                             % SIGNED: negative in reverse
v(1)   = 0;                                 % start from rest
v(end) = 0;                                 % end at rest

% Zero at cusps - M3's profiler does this, so reproduce it faithfully.
cuspMask = [false; diff(d) ~= 0];
if any(cuspMask)
    ci = find(cuspMask);
    v(ci) = 0;
    v(max(ci-1,1)) = 0;                     % zero both sides of the cusp
end

%% ---- Pack into the fixed-width bus -----------------------------------
plan = struct();
plan.Waypoints           = zeros(MAXWP, 4);
plan.Waypoints(1:n, :)   = [x, y, th, v];
plan.NumWaypoints        = uint32(n);       % uint32, per M3's contract
plan.Directions          = zeros(MAXWP, 1);
plan.Directions(1:n)     = d;               % TODO: confirm type with M3
plan.SeqNum              = uint32(seqNum);  % uint32, per M3's contract
plan.GenerationTimestamp = nowSec;
plan.MapTimestamp        = nowSec - 0.08;
plan.MapAgeAtPlan_s      = 0.08;
end

% -----------------------------------------------------------------------
function k = localCurvature(x, y)
dx  = gradient(x);   dy  = gradient(y);
ddx = gradient(dx);  ddy = gradient(dy);
den = (dx.^2 + dy.^2).^1.5;
k   = (dx.*ddy - dy.*ddx) ./ max(den, 1e-9);
end
