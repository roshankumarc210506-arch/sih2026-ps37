function dets = sihLidarClusterDetections(lidarSensor, egoVehicle, poses, time, cfg, classOf)
%SIHLIDARCLUSTERDETECTIONS  Real LiDAR point cloud -> per-object detections.
%
%   dets = sihLidarClusterDetections(lidarSensor, egoVehicle, poses, time, cfg, classOf)
%
%   Unlike vision/radar, lidarPointCloudGenerator returns a raw point
%   cloud, not object-level detections — this function does the
%   clustering. Output format matches vision/radar exactly (cell array
%   of objectDetection, SensorIndex=3), so nothing downstream changes.
%
%   VERIFIED Day 3 (empirically, not assumed):
%     1. ptCloud.Location IS ego-frame, confirmed by checking the closest
%        point to a known nearby target's true position (0.57m, matching
%        actor half-extent) and confirming point ranges stay bounded by
%        cfg.Sensor(3).MaxRange rather than tracking ego's own drifting
%        world position. CONFIRMED CORRECT.
%     2. pcsegdist's label convention — confirmed by reading
%        pcsegdist.m's actual implementation (help pcsegdist itself
%        returns nothing useful, just a copyright line): labels is
%        zero-initialized, invalid/no-return points and undersized
%        clusters are left/reset to 0, valid clusters are numbered from
%        1. `labels > 0` for real clusters is CONFIRMED CORRECT.
%     3. FOUND AND FIXED: ptCloud.Location from an organized sensor is
%        [rows x cols x 3] (elevation x azimuth x xyz), NOT a flat Nx3
%        matrix. The original pts(mask,:) crashed outright ("logical
%        indices ... outside of array bounds") the moment this was
%        wired into the real pipeline — not a silent bug, a hard error
%        on the first LiDAR detection. Fixed by reshaping both pts and
%        labels to flat Nx3/Nx1 immediately after retrieval.
%     4. minDistance/minPoints below are reasoned defaults, not measured
%        — smallest actor footprint is ~0.9x0.6m (PushCart/Pedestrian),
%        0.6m clustering distance should keep single objects merged
%        without bridging distinct nearby actors, but this needs
%        checking against REAL point density, not just object size.

dets = {};

rdmesh = roadMesh(egoVehicle);
[ptCloud, isValidTime] = lidarSensor(poses, rdmesh, time);

if ~isValidTime || ptCloud.Count == 0
    return
end

% ptCloud.Location from an organized sensor like lidarPointCloudGenerator
% is [rows x cols x 3] (elevation x azimuth x xyz), NOT a flat Nx3 matrix
% -- confirmed empirically (size was [32 720 3] for this sensor config).
% pts(mask,:) below needs both flattened to line up; pcsegdist's `labels`
% output matches ptCloud.Location's organized shape too ([rows x cols]),
% same reshape needed there.
pts = reshape(ptCloud.Location, [], 3);
if isempty(pts)
    return
end

minDistance = 0.6;   % m, TUNABLE — see header note (3)
labels = reshape(pcsegdist(ptCloud, minDistance), [], 1);

minPoints = 5;       % clusters smaller than this are treated as noise
maxPoints = 200;      % clusters larger than this are treated as ground/
                       % background, not an actor -- confirmed empirically:
                       % every frame in a 248-frame run had a ~9822-point
                       % static ground-plane cluster (150x+ larger than any
                       % real actor cluster, max observed 55), passing
                       % through as a phantom detection every single frame.
uniqueLabels = unique(labels(labels > 0));

% Ground truth positions, for nearest-actor attribution (class-confusion
% purposes only — this does NOT feed the tracker, purely for deciding
% which class label to attach, same role classOf plays for vision/radar).
truePos = zeros(numel(poses), 2);
for k = 1:numel(poses)
    truePos(k,:) = poses(k).Position(1:2);
end

for L = uniqueLabels(:)'
    mask = (labels == L);
    if nnz(mask) < minPoints || nnz(mask) > maxPoints
        continue
    end

    clusterPts = pts(mask, :);
    centroid   = mean(clusterPts, 1);   % [x y z], assumed ego-frame

    actorID = -1;
    if ~isempty(poses)
        d = hypot(truePos(:,1) - centroid(1), truePos(:,2) - centroid(2));
        [dmin, idx] = min(d);
        if dmin < 2.0   % gate: don't attribute to a wildly distant actor
            actorID = poses(idx).ActorID;
        end
    end

    if actorID > 0 && isKey(classOf, actorID)
        trueClass = classOf(actorID);
        if rand < cfg.Sensor(3).ClassAcc
            reported = trueClass;
        else
            reported = sihConfuseClass(trueClass, cfg);
        end
    else
        reported = AgentClass.Unknown;
    end

    R = diag([0.05^2, 0.05^2, cfg.MeasZNoise]);   % LiDAR is precise;
    % TUNABLE — reconcile against cfg.Sensor(3).PosStd once real
    % point-cloud accuracy is observed empirically, same open item as
    % AzimuthResolution/ElevationLimits in sihCreateLidar.m.

    % ObjectAttributes field set must match vision/radar's EXACTLY
    % (TruthActorID, TruthClass, SensorName only) -- confirmed the hard
    % way: multiObjectTracker concatenates a track's ObjectAttributes
    % history across whichever sensors touched it, and struct field-set
    % mismatches (this originally also carried ClusterPoints) crash with
    % "Number of fields in structure arrays being concatenated do not
    % match" deep inside ObjectTrack/get.ObjectAttributes. Cluster size
    % is reported separately (see run summary), not carried on the
    % detection.
    dets{end+1} = objectDetection(time, centroid(:), ...          %#ok<AGROW>
        'MeasurementNoise', R, ...
        'SensorIndex',      3, ...
        'ObjectClassID',    double(reported), ...
        'ObjectAttributes', {struct('TruthActorID', actorID, ...
                                     'TruthClass',   double(reported), ...
                                     'SensorName',   'LiDAR')});
end

dets = dets(:);
end
