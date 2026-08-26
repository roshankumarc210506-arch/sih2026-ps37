function velPlan = assignVelocityProfile(planResult, cfg, externalSpeedCap_mps)
%ASSIGNVELOCITYPROFILE  Signed velocity + per-waypoint timing.
%
%   Two-pass (forward accel-limited, backward decel-limited) profile,
%   bounded above by a per-waypoint curvature cap. Segmented at cusps:
%   velocity is forced to zero at every direction change, and at the
%   path's start/goal per cfg.Vel.StartSpeed_mps / GoalSpeed_mps.
%
%   externalSpeedCap_mps (optional) - a speed ceiling supplied by an
%   external caller (e.g. M4's driving_mode consumer, per the locked
%   interface contract: "driving_mode ... caps velocity profiler speed
%   limit"). Defaults to cfg.Vel.MaxSpeed_mps, i.e. no additional cap.
%   NOTE: the signal/bus that carries this value is not yet named in the
%   locked contract - flagged for M4/M5 to define. This parameter is the
%   integration point once it is.

if nargin < 3 || isempty(externalSpeedCap_mps)
    externalSpeedCap_mps = cfg.Vel.MaxSpeed_mps;
end

S = planResult.States;
D = double(planResult.Directions(:));   % toolbox may return int8; cast for arithmetic
N = size(S,1);

if N < 2
    error('assignVelocityProfile:TooFewWaypoints', 'Need >= 2 waypoints.');
end

%% ---------- arc length between consecutive waypoints ----------
dxy = diff(S(:,1:2), 1, 1);
ds  = vecnorm(dxy, 2, 2);           % N-1 segment lengths
ds(ds < 1e-9) = 1e-9;                % guard divide-by-zero on duplicate points

%% ---------- curvature at each waypoint ----------
% kappa = |dtheta|/ds, wrapped manually (no toolbox dependency assumed).
theta  = S(:,3);
dtheta = mod(diff(theta) + pi, 2*pi) - pi;      % wrap to (-pi, pi]
kappaSeg = abs(dtheta) ./ ds;                   % N-1, one per segment

kappa = zeros(N,1);
kappa(1)   = kappaSeg(1);
kappa(end) = kappaSeg(end);
kappa(2:end-1) = 0.5*(kappaSeg(1:end-1) + kappaSeg(2:end));

%% ---------- segment boundaries: split at every cusp ----------
cuspIdx   = planResult.CuspIndices(:);
segBounds = unique([1; cuspIdx; N]);

%% ---------- per-waypoint speed cap: curvature + global max + external cap ----------
vMaxGlobal = min(cfg.Vel.MaxSpeed_mps, externalSpeedCap_mps);
vCap = vMaxGlobal * ones(N,1);
highCurv = kappa > cfg.Vel.CurvatureEps;
vCap(highCurv) = min(vMaxGlobal, sqrt(cfg.Vel.LatAccelCap_mps2 ./ kappa(highCurv)));

%% ---------- two-pass accel/decel-limited profile, per segment ----------
vAbs = zeros(N,1);
nSeg = numel(segBounds) - 1;

for si = 1:nSeg
    i0  = segBounds(si);
    i1  = segBounds(si+1);
    idx = i0:i1;
    n   = numel(idx);

    vSeg = vCap(idx);
    if D(i0) < 0
        vSeg = min(vSeg, cfg.Vel.MaxReverse_mps);
    end

    v = vSeg;
    v(1)   = (si == 1)    * cfg.Vel.StartSpeed_mps;   % 0 unless first segment start
    v(end) = (si == nSeg) * cfg.Vel.GoalSpeed_mps;     % 0 unless final segment end

    % forward pass: v(k) reachable from v(k-1) under LongAccelCap
    for k = 2:n
        vLim = sqrt(v(k-1)^2 + 2*cfg.Vel.LongAccelCap_mps2*ds(idx(k-1)));
        v(k) = min(v(k), vLim);
    end
    % backward pass: v(k) must allow stopping/slowing into v(k+1) under LongDecelCap
    for k = n-1:-1:1
        vLim = sqrt(v(k+1)^2 + 2*cfg.Vel.LongDecelCap_mps2*ds(idx(k)));
        v(k) = min(v(k), vLim);
    end

    vAbs(idx) = v;
end

%% ---------- signed velocity ----------
vSigned = vAbs .* sign(D);
vSigned(vAbs < 1e-9) = 0;    % clean exact zero at cusps regardless of sign(D)

%% ---------- per-waypoint time ----------
% Trapezoidal ds/v_avg, with constant-accel kinematics as the special case
% when both endpoints of a segment are (numerically) at rest.
t = zeros(N,1);
for k = 2:N
    v1 = abs(vSigned(k-1));
    v2 = abs(vSigned(k));
    vAvg = 0.5*(v1+v2);
    if vAvg < 1e-6
        dt = sqrt(2*ds(k-1) / max(cfg.Vel.LongAccelCap_mps2, 1e-3));
    else
        dt = ds(k-1) / vAvg;
    end
    t(k) = t(k-1) + dt;
end

%% ---------- assemble ----------
velPlan = struct();
velPlan.Waypoints    = [S(:,1), S(:,2), S(:,3), vSigned];   % N x 4, per spec
velPlan.Directions   = D;
velPlan.Time_s       = t;              % diagnostic; NOT in the Step-6 bus schema
velPlan.Curvature    = kappa;
velPlan.ArcLength_m  = [0; cumsum(ds)];
velPlan.NumWaypoints = N;
velPlan.CuspIndices  = cuspIdx;
velPlan.MaxSpeedReached_mps = max(abs(vSigned));
velPlan.TotalTime_s  = t(end);
velPlan.SpeedCapApplied_mps = vMaxGlobal;   % audit trail: what cap was actually in force
end