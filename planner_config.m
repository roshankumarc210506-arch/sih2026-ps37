function cfg = planner_config()
%PLANNER_CONFIG  Single source of truth for the AV planning stack.
%
%   Every tunable number lives here. No magic constants downstream.
%   Geometry convention: all SE(2) states are REAR-AXLE referenced.

%% ===================== VEHICLE =====================
cfg.Veh.Wheelbase_m      = 2.50;
cfg.Veh.Length_m         = 4.70;
cfg.Veh.Width_m          = 1.80;
cfg.Veh.RearOverhang_m   = 0.90;
cfg.Veh.MaxSteer_rad     = deg2rad(35);

% Derived geometry (do not hand-edit; recomputed below)
cfg.Veh.FrontOverhang_m  = cfg.Veh.Length_m - cfg.Veh.Wheelbase_m - cfg.Veh.RearOverhang_m;
cfg.Veh.FrontAxleToBumper_m = cfg.Veh.Wheelbase_m + cfg.Veh.FrontOverhang_m;   % 3.80 m from rear axle
cfg.Veh.CentreOffset_m   = cfg.Veh.Length_m/2 - cfg.Veh.RearOverhang_m;        % 1.45 m from rear axle

%% ===================== FOOTPRINT =====================
% 'single'      : one disc, radius cfg.Foot.Radius_m, stamped at the state point
% 'threeCircle' : three discs along the body axis (correct, far less conservative)
cfg.Foot.Mode            = 'single';

% --- single-disc mode ---
cfg.Foot.Radius_m        = 2.52;    % as specified. NOTE: centre-referenced value.
% Rear-axle-correct value is 3.91 m.
% Step 7 audit reports actual clearance.

% --- three-circle mode (rear-axle referenced, exact cover of the 4.7x1.8 body) ---
cfg.Foot.NumCircles      = 3;
cfg.Foot.CircleRadius_m  = 1.1932;                        % sqrt(0.7833^2 + 0.9^2)
cfg.Foot.CircleOffsets_m = [-0.1167, 1.4500, 3.0167];     % along heading, from rear axle

%% ===================== MAP =====================
cfg.Map.Resolution_cpm   = 4;       % cells per metre -> 0.25 m cells
cfg.Map.Width_m          = 60;
cfg.Map.Height_m         = 40;

%% ===================== PER-CLASS RISK INFLATION =====================
% Extra inflation ON TOP of the ego footprint radius, per obstacle class.
% Unstructured Indian roads: vulnerable/erratic agents get more room.
cfg.Risk.static_m        = 0.00;    % walls, kerbs, parked structures
cfg.Risk.vehicle_m       = 0.30;    % cars, autos, trucks
cfg.Risk.pedestrian_m    = 0.75;    % people
cfg.Risk.animal_m        = 0.90;    % cattle, dogs - least predictable

%% ===================== PLANNER =====================
cfg.Plan.MinTurningRadius_m       = 4.10;   % 1.15 x (2.5/tan(35deg)) = 1.15 x 3.57
cfg.Plan.MotionPrimitiveLength_m  = 2.00;   % >= 4 cells at 0.25 m
cfg.Plan.NumMotionPrimitives      = 5;      % odd -> includes straight-ahead
cfg.Plan.ForwardCost              = 1.0;
cfg.Plan.ReverseCost              = 3.0;    % discourage but permit reverse
cfg.Plan.DirectionSwitchingCost   = 20.0;   % cusps are expensive (each = full stop)
cfg.Plan.AnalyticExpansionInterval= 5;      % Reeds-Shepp shot every 5 expansions
cfg.Plan.InterpolationDistance_m  = 0.25;   % output waypoint spacing = 1 cell
cfg.Plan.MotionDirection          = "forward-reverse"; % forward + reverse -> cusps possible
cfg.Plan.MaxNumNodes              = 1e5;
cfg.Plan.MaxNumPathStates         = 5e3;
cfg.Plan.SearchMode               = "greedy";
cfg.Plan.ValidationDistance_m     = 0.10;   % < 0.25 m cell. Default Inf steps over obstacles.

%% ===================== VELOCITY PROFILER =====================
cfg.Vel.MaxSpeed_mps      = 8.0;    % ~29 km/h, appropriate for unstructured road
cfg.Vel.MaxReverse_mps    = 1.5;
cfg.Vel.LatAccelCap_mps2  = 1.5;    % v <= sqrt(a_lat / kappa)
cfg.Vel.LongAccelCap_mps2 = 1.0;
cfg.Vel.LongDecelCap_mps2 = 1.5;    % braking authority > accel authority
cfg.Vel.StartSpeed_mps    = 0.0;
cfg.Vel.GoalSpeed_mps     = 0.0;
cfg.Vel.CurvatureEps      = 1e-4;   % below this, treat as straight

%% ===================== BUS / TIMING =====================
cfg.Bus.MaxWaypoints      = 5000;   % must be >= cfg.Plan.MaxNumPathStates
cfg.Bus.MaxMapAge_s       = 0.50;   % NMPC should reject plans older than this

%% ===================== CONSISTENCY CHECKS =====================
assert(cfg.Veh.FrontOverhang_m > 0, 'Vehicle dimensions inconsistent.');
assert(cfg.Bus.MaxWaypoints >= cfg.Plan.MaxNumPathStates, ...
    'Bus.MaxWaypoints (%d) < Plan.MaxNumPathStates (%d).', ...
    cfg.Bus.MaxWaypoints, cfg.Plan.MaxNumPathStates);
assert(cfg.Plan.ValidationDistance_m < 1/cfg.Map.Resolution_cpm, ...
    'ValidationDistance (%.3f) must be < cell size (%.3f).', ...
    cfg.Plan.ValidationDistance_m, 1/cfg.Map.Resolution_cpm);
assert(mod(cfg.Plan.NumMotionPrimitives,2) == 1, ...
    'NumMotionPrimitives should be odd so a straight primitive exists.');

% Kinematic feasibility of the stated turning radius
R_kin = cfg.Veh.Wheelbase_m / tan(cfg.Veh.MaxSteer_rad);
assert(cfg.Plan.MinTurningRadius_m >= R_kin, ...
    'MinTurningRadius %.2f < kinematic minimum %.2f.', ...
    cfg.Plan.MinTurningRadius_m, R_kin);
end

