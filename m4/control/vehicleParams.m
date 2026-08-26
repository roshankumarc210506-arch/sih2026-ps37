function vp = vehicleParams()
%VEHICLEPARAMS Locked vehicle constants for SIH2026 PS37. Single source of truth.
%
%   Every module that needs a vehicle number reads it from HERE. If this
%   file and the interface contract ever disagree, the contract wins and
%   this file gets fixed - never the other way round.
%
%   Values locked per SIH2026_PS37_Task_Split contract block.

%% ---- Locked geometry -------------------------------------------------
vp.L            = 2.5;              % wheelbase [m]
vp.length       = 4.7;              % body length [m]
vp.width        = 1.8;              % body width [m]
vp.rearOverhang = 0.9;              % rear axle to rear bumper [m]
vp.deltaMax     = deg2rad(35);      % max front steering angle [rad]

% Reference point for all states is the REAR AXLE (matches bicycleStateFcn).
vp.frontOverhang = vp.length - vp.rearOverhang - vp.L;   % 1.3 m
vp.noseFromAxle  = vp.length - vp.rearOverhang;          % 3.8 m
vp.tailFromAxle  = -vp.rearOverhang;                     % -0.9 m

%% ---- Turning radii ---------------------------------------------------
vp.Rmin_kinematic = vp.L / tan(vp.deltaMax);   % 3.570 m - physical limit
vp.Rmin_planner   = 4.10;                      % locked: kinematic + 15% margin

% M3 plans with the LARGER radius, so every path handed to us is strictly
% inside our physical envelope. That is a gift: reference infeasibility
% should never be the root cause of a tracking failure. If tracking fails,
% look at weights/constraints, not at whether the path is drivable.

%% ---- Footprint: 3-disc model ----------------------------------------
% M3's collision check uses ONE disc of radius 2.52 m centred at the rear
% axle. Audited under-coverage at the nose: see below. Our NMPC obstacle
% constraints use a 3-disc model instead, which fully covers the rectangle
% with a much smaller radius - tighter AND safer at the same time.
n = 3;
vp.nDiscs     = n;
vp.discRadius = hypot(vp.length/(2*n), vp.width/2);          % 1.1932 m
vp.discOffset = vp.tailFromAxle + vp.length*(2*(1:n)-1)/(2*n);
%                                  -> [-0.1167, 1.4500, 3.0167] m from rear axle

%% ---- Audit of M3's single-disc footprint -----------------------------
vp.singleDiscRadius   = 2.52;
vp.noseUnderCoverage  = hypot(vp.noseFromAxle, vp.width/2) - vp.singleDiscRadius;
%   = hypot(3.8, 0.9) - 2.52 = 3.9051 - 2.52 = 1.3851 m
%
%   Confirms M3's flagged 1.385 m figure exactly. Consequence for US:
%   the planner will happily route the front corners of the vehicle through
%   an obstacle. Our MPC soft constraints are the ONLY thing covering that
%   gap, so the forward discs must not be disabled for speed.

%% ---- Derived comfort limits ------------------------------------------
vp.aLatMax  = 2.0;      % lateral accel cap [m/s^2] - matches M3's profiler
vp.aLongMax = 2.0;      % max forward accel [m/s^2]
vp.aLongMin = -3.5;     % max braking [m/s^2]

end
