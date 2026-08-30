function lidarSensor = sihCreateLidar(scenario, egoVehicle, cfg)
%SIHCREATELIDAR  Day 3: real LiDAR point-cloud generator.
%
%   lidarSensor = sihCreateLidar(scenario, egoVehicle, cfg)
%
%   Calling convention verified against MathWorks docs (matched
%   identically across 5 separate doc pages, not a single source):
%     tgts   = targetPoses(egoVehicle);
%     rdmesh = roadMesh(egoVehicle);
%     [ptCloud, isValidTime] = lidarSensor(tgts, rdmesh, time);
%
%   Unlike visionDetectionGenerator/drivingRadarDataGenerator, this
%   returns a raw POINT CLOUD, not object-level detections — clustering
%   into per-object detections happens in sihRealDetections.m, not here.
%
%   Requires every actor to have a Mesh property set (sihActorMesh.m) —
%   ray tracing has nothing to bounce rays off otherwise. Confirmed
%   Day 3 foundation work: mesh assignment verified to have zero effect
%   on Day 2's camera/radar regression numbers.

lidarSensor = lidarPointCloudGenerator( ...
    'UpdateInterval',    cfg.SampleTime, ...
    'ActorProfiles',     actorProfiles(scenario), ...
    'EgoVehicleActorID', egoVehicle.ActorID, ...
    'MaxRange',          cfg.Sensor(3).MaxRange, ...
    'AzimuthResolution', 0.5, ...
    'ElevationLimits',   [-25 15]);
    % AzimuthResolution/ElevationLimits are reasonable automotive LiDAR
    % defaults (comparable to a Velodyne-class sensor), not tuned to
    % anything in cfg yet — cfg.Sensor(3) currently only carries the
    % Day-1 stub's statistical params (PosStd, Pd, ClassAcc etc.), which
    % don't map cleanly onto ray-tracing parameters. Worth reconciling
    % once real point-cloud output is validated against those numbers.

fprintf('[M1] Real LiDAR created: MaxRange=%dm, ego ActorID=%d\n', ...
    cfg.Sensor(3).MaxRange, egoVehicle.ActorID);
end
