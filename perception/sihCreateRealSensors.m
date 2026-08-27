function [visionSensor, radarSensor] = sihCreateRealSensors(scenario, egoVehicle, cfg)
%SIHCREATEREALSENSORS  Day 2: real camera + radar models.
%
%   [vision, radar] = sihCreateRealSensors(scenario, egoVehicle, cfg)
%
%   Replaces Day 1's hand-rolled statistics in sihDummyDetections.m for
%   these two sensors. LiDAR stays a statistical stub until Day 3 —
%   point-cloud clustering is a separate, bigger piece of work and
%   swapping all three at once is how Day 1 sprawled.
%
%   Verified against MathWorks docs before writing, not guessed:
%     - visionDetectionGenerator's FieldOfView is DERIVED from Intrinsics,
%       not settable directly. We use realistic automotive intrinsics
%       rather than force-fitting Day 1's 60 deg guess. The resulting
%       ~44 deg H-FOV is the honest number — quote THIS to M3/M5 going
%       forward, not the old placeholder.
%     - drivingRadarDataGenerator's FieldOfView IS settable directly, so
%       cfg.Sensor(2).FoV carries over exactly.
%     - Both take targetPoses(egoVehicle) output directly — no change to
%       the existing poses-based loop structure needed.
%     - Both explicitly support RoadRunner Scenario as well as
%       drivingScenario, so this is unaffected by the open
%       RoadRunner-vs-DSD decision.
%
%   Mounting locations are computed from egoVehicle's ACTUAL dimensions
%   (Wheelbase, FrontOverhang), not hardcoded, so they track reality if
%   the vehicle geometry changes. Answers the mounting question M5 asked:
%   camera near the windshield, radar at the front bumper — not
%   collocated at the rear axle like the Day 1 stub implicitly was.

% Exclude ego's own actor profile by ActorID match, not by assuming index 1
allProfiles = actorProfiles(scenario);
targetProfs = allProfiles([allProfiles.ActorID] ~= egoVehicle.ActorID);

% ---------------- Camera ----------------
camForward = 0.75 * egoVehicle.Wheelbase;   % near windshield/mirror

visionSensor = visionDetectionGenerator( ...
    'SensorIndex',             1, ...
    'UpdateInterval',          cfg.SampleTime, ...
    'SensorLocation',          [camForward, 0], ...
    'Height',                  1.1, ...
    'Pitch',                   0, ...
    'Intrinsics',              cameraIntrinsics(800, [320 240], [480 640]), ...
    'MaxRange',                cfg.Sensor(1).MaxRange, ...
    'DetectionProbability',    cfg.Sensor(1).Pd, ...
    'FalsePositivesPerImage',  cfg.Sensor(1).FalseAlarmRate, ...
    'ActorProfiles',           targetProfs);

% ---------------- Radar ----------------
radForward = egoVehicle.FrontOverhang;      % front bumper

radarSensor = drivingRadarDataGenerator(2, ...
    'UpdateRate',           1/cfg.SampleTime, ...
    'MountingLocation',     [radForward, 0, 0.5], ...
    'FieldOfView',          [cfg.Sensor(2).FoV, 5], ...
    'RangeLimits',          [0, cfg.Sensor(2).MaxRange], ...
    'DetectionProbability', cfg.Sensor(2).Pd, ...
    'Profiles',             targetProfs);
    % FalseAlarmRate deliberately left at its real default (~1e-6).
    % It's a per-resolution-cell detection-theoretic probability, NOT
    % the same quantity as Day 1's cfg.Sensor(2).FalseAlarmRate (which
    % meant "expected false alarms per frame" — many orders of magnitude
    % larger). Reusing that number here would be wrong, not just
    % different. Expect real false-track pressure to drop as a result.

fprintf('[M1] Real sensors created. Camera FOV %.1f x %.1f deg (from intrinsics, was an unvalidated 60 deg guess in the stub). Radar FOV %d x 5 deg.\n', ...
    visionSensor.FieldOfView(1), visionSensor.FieldOfView(2), cfg.Sensor(2).FoV);
end
