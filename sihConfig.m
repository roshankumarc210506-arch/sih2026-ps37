function cfg = sihConfig()
%SIHCONFIG  Single source of truth for M1 (Perception) parameters.
%   Everything tunable lives here so Phase 1 tuning is edit-one-file.
%
%   cfg = sihConfig();

% ---------- Timing ----------
cfg.SampleTime = 0.1;      % s   (10 Hz perception update)
cfg.StopTime   = 25;       % s

% ---------- Bus sizing (Simulink needs FIXED sizes) ----------
cfg.MaxTracks  = 20;       % fixed-length track array in the bus
cfg.NumClasses = 7;        % Unknown + 6 real classes

% ---------- Frame convention ----------
% Ego-vehicle frame, ISO 8855: x forward, y LEFT, theta CCW from +x.
% All perception output is in EGO coordinates. Ego pose in world is
% published on a separate bus so M3/M6 can transform if they need world.
cfg.Frame = 'ego_ISO8855';

% ---------- Sensor models (Day 1 = statistical stubs) ----------
% Index 1 = Camera, 2 = Radar, 3 = LiDAR.
% PosStd is [longitudinal_std, lateral_std] in metres — this asymmetry is
% what makes fusion actually worth doing (camera is bad at range, radar is
% bad at cross-range, lidar is good at both).

cfg.Sensor(1).Name           = 'Camera';
cfg.Sensor(1).PosStd         = [1.60 0.28];   % m  (poor range, good bearing)
cfg.Sensor(1).MaxRange       = 60;            % m
cfg.Sensor(1).FoV            = 60;            % deg (total)
cfg.Sensor(1).Pd             = 0.90;          % detection probability
cfg.Sensor(1).ClassAcc       = 0.88;          % P(correct class label)
cfg.Sensor(1).FalseAlarmRate = 0.05;          % expected FAs per frame

cfg.Sensor(2).Name           = 'Radar';
cfg.Sensor(2).PosStd         = [0.30 1.40];   % m  (good range, poor cross-range)
cfg.Sensor(2).MaxRange       = 100;
cfg.Sensor(2).FoV            = 40;
cfg.Sensor(2).Pd             = 0.93;
cfg.Sensor(2).ClassAcc       = 0.25;          % radar barely classifies
cfg.Sensor(2).FalseAlarmRate = 0.15;

cfg.Sensor(3).Name           = 'LiDAR';
cfg.Sensor(3).PosStd         = [0.14 0.14];   % m
cfg.Sensor(3).MaxRange       = 50;
cfg.Sensor(3).FoV            = 360;
cfg.Sensor(3).Pd             = 0.95;
cfg.Sensor(3).ClassAcc       = 0.70;          % geometry-based, confuses similar sizes
cfg.Sensor(3).FalseAlarmRate = 0.08;

cfg.MeasZNoise = 0.05;    % m^2, keeps the (unused) z channel from drifting

% ---------- Tracker tuning ----------
cfg.Tracker.AssignmentThreshold  = 40;    % normalised distance gate
cfg.Tracker.ConfirmationThreshold = [3 5];% M-of-N to confirm
cfg.Tracker.DeletionThreshold     = [5 5];% M-of-N to delete
cfg.Tracker.ClutterDensity        = 1e-6; % JPDA only
cfg.Tracker.InitVelStd            = 6;    % m/s, initial velocity uncertainty

% ---------- Contract-format extras ----------
cfg.MinSpeedForHeading = 0.25;  % m/s below this, heading is unreliable
cfg.StationaryHeadingVar = (pi/3)^2; % rad^2 injected when nearly stationary

% ---------- Class confusion (stub only; realistic pairs) ----------
% Which class a sensor is most likely to mistake a given class for.
cfg.ConfusionPairs = { ...
    AgentClass.Car,          [AgentClass.AutoRickshaw, AgentClass.Unknown]; ...
    AgentClass.TwoWheeler,   [AgentClass.Pedestrian,   AgentClass.AutoRickshaw]; ...
    AgentClass.AutoRickshaw, [AgentClass.Car,          AgentClass.TwoWheeler]; ...
    AgentClass.PushCart,     [AgentClass.Animal,       AgentClass.Unknown]; ...
    AgentClass.Pedestrian,   [AgentClass.TwoWheeler,   AgentClass.Unknown]; ...
    AgentClass.Animal,       [AgentClass.PushCart,     AgentClass.Pedestrian]};

% ---------- Reproducibility ----------
cfg.RandomSeed = 37;   % PS number, why not
end
