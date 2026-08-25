function dets = sihDummyDetections(poses, time, cfg, classOf)
%SIHDUMMYDETECTIONS  Day-1 stub replacement for camera/radar/LiDAR models.
%
%   dets = sihDummyDetections(targetPoses(ego), time, cfg, classOf)
%
%   Produces a cell array of objectDetection in EGO coordinates, with
%   per-sensor: field of view, max range, detection probability, anisotropic
%   position noise, false alarms, and class confusion.
%
%   This is deliberately drop-in compatible with the real Day-2 models:
%     Camera -> visionDetectionGenerator
%     Radar  -> drivingRadarDataGenerator
%     LiDAR  -> lidarPointCloudGenerator + clustering
%   All three produce the same objectDetection cell array, so nothing
%   downstream of this function changes when you swap them in.

dets = {};

for s = 1:numel(cfg.Sensor)
    S = cfg.Sensor(s);

    % ---- true targets ----
    for k = 1:numel(poses)
        p  = poses(k).Position;          % [x y z], ego frame
        r  = hypot(p(1), p(2));
        az = atan2d(p(2), p(1));

        if r > S.MaxRange,          continue; end
        if abs(az) > S.FoV/2,       continue; end

        % Range-degraded detection probability
        pd = S.Pd * max(0.35, 1 - 0.5*(r/S.MaxRange)^2);
        if rand > pd,               continue; end

        R = diag([S.PosStd(1)^2, S.PosStd(2)^2, cfg.MeasZNoise]);
        meas = [ p(1) + S.PosStd(1)*randn
                 p(2) + S.PosStd(2)*randn
                 0 ];

        trueClass = classOf(poses(k).ActorID);
        if rand < S.ClassAcc
            reported = trueClass;
        else
            reported = sihConfuseClass(trueClass, cfg);
        end

        dets{end+1} = objectDetection(time, meas, ...           %#ok<AGROW>
            'MeasurementNoise', R, ...
            'SensorIndex',      s, ...
            'ObjectClassID',    double(reported), ...
            'ObjectAttributes', {struct( ...
                'TruthActorID', poses(k).ActorID, ...
                'TruthClass',   double(trueClass), ...
                'SensorName',   S.Name)});
    end

    % ---- false alarms (clutter: dust, potholes, roadside junk) ----
    if rand < S.FalseAlarmRate
        rr   = 5 + (S.MaxRange-5)*rand;
        aa   = deg2rad((rand-0.5)*min(S.FoV,180));
        meas = [rr*cos(aa); rr*sin(aa); 0];
        R    = diag([S.PosStd(1)^2, S.PosStd(2)^2, cfg.MeasZNoise]);

        dets{end+1} = objectDetection(time, meas, ...            %#ok<AGROW>
            'MeasurementNoise', R, ...
            'SensorIndex',      s, ...
            'ObjectClassID',    double(AgentClass.Unknown), ...
            'ObjectAttributes', {struct( ...
                'TruthActorID', -1, ...
                'TruthClass',   double(AgentClass.Unknown), ...
                'SensorName',   S.Name)});
    end
end

dets = dets(:);
end

% ------------------------------------------------------------------------
function c = sihConfuseClass(trueClass, cfg)
%SIHCONFUSECLASS  Pick a plausible wrong label instead of a uniform-random one.
c = AgentClass.Unknown;
for i = 1:size(cfg.ConfusionPairs,1)
    if cfg.ConfusionPairs{i,1} == trueClass
        opts = cfg.ConfusionPairs{i,2};
        c    = opts(randi(numel(opts)));
        return
    end
end
end
