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
%     4. minDistance/minPoints: 0.6m was the unmeasured reasoned default.
%        Day 4 first tried 0.75m based on real OverSegmented evidence
%        (137 instances at 0.6m, median size 6 points) -- caused a severe
%        regression (false tracks nearly doubled, every diagnostic bucket
%        read 0) and was reverted. Added a raw-cluster-count-per-frame
%        diagnostic (sihLidarDiag RawClusterCounts) to establish a real
%        baseline (mean 18.66/frame at 0.6m) before trying again. Now on
%        a second, much smaller attempt at 0.65m -- see the constant's
%        own comment below for the full reasoning and what to check.
%     5. Day 4 (v5): restructured to compute the cluster centroid and run
%        the actor-attribution gate BEFORE any accept/reject branching
%        (moved up from after the size filter), since two new diagnostic
%        paths (FilteredTooSmall/FilteredTooBig, MatchedPosError) also
%        need them. The actor-gate distance check itself (nearest real
%        actor within 2.0m of the centroid) was needed a third time --
%        ground-rejection attribution, class-confusion attribution, and
%        now the FilteredTooSmall diagnostic -- so it's extracted into
%        the local sihLidarNearestActor helper below instead of being
%        duplicated again. All of this is purely observational: it does
%        NOT change which detections get emitted to the tracker.

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

minDistance = 0.60;  % m, 2nd attempt. 0.6 baseline raw cluster count
                      % measured first (mean 18.66/frame, median 19, min 8,
                      % max 30 -- see sihLidarDiag RawClusterCounts). Prior
                      % attempt jumped to 0.75 (25% increase) and caused a
                      % severe regression (every diagnostic bucket read 0,
                      % false tracks nearly doubled) -- likely merged most/
                      % all of the ground plane into oversized clusters
                      % that blew past maxPoints=200 before reaching any
                      % tagging logic. This time: a much smaller, ~8%
                      % increase, specifically to avoid repeating that
                      % failure mode. CHECK THE RAW CLUSTER COUNT FIRST in
                      % the new run's output -- if it collapses sharply
                      % from the 0.6 baseline above, stop and revert
                      % immediately rather than reading further into the
                      % other diagnostics, same as last time.
labels = reshape(pcsegdist(ptCloud, minDistance), [], 1);

minPoints = 5;       % clusters smaller than this are treated as noise.
                       % Day 4: tried lowering to 3 based on sihLidarDiag
                       % showing 84.6% of FilteredTooSmall clusters had a
                       % real actor within the attribution gate (concentrated
                       % in TwoWheeler/Pedestrian) -- reverted after an A/B
                       % test: false tracks/frame rose 2.67->3.19 (+19.5%)
                       % and classification accuracy fell 63.4%->59.7%
                       % (-3.7pp), while the target classes barely moved
                       % (TwoWheeler per-actor tracking 78.6%->78.2%, flat).
                       % Most of the newly-admitted small clusters turned
                       % into duplicate/split detections on already-tracked
                       % objects (OverSegmented +25%), not rescues of missed
                       % ones. Proximity-to-a-real-actor is not the same as
                       % being that actor's detection -- the diagnostic
                       % measured the right thing but doesn't prove
                       % causation on its own; keep 5 unless a future
                       % attempt addresses the OverSegmented side effect.
maxPoints = 200;      % clusters larger than this are treated as ground/
                       % background, not an actor -- confirmed empirically:
                       % every frame in a 248-frame run had a ~9822-point
                       % static ground-plane cluster (150x+ larger than any
                       % real actor cluster, max observed 55), passing
                       % through as a phantom detection every single frame.
uniqueLabels = unique(labels(labels > 0));
sihLidarDiag('recordFrameCount', numel(uniqueLabels));   % (v4) DIAGNOSTIC:
% raw cluster count BEFORE minPoints/maxPoints filtering, this frame.
% See sihLidarDiag.m header -- added to see pcsegdist's actual behavior
% before touching minDistance again.

% Ground truth positions, for nearest-actor attribution (class-confusion
% purposes only — this does NOT feed the tracker, purely for deciding
% which class label to attach, same role classOf plays for vision/radar).
truePos = zeros(numel(poses), 2);
for k = 1:numel(poses)
    truePos(k,:) = poses(k).Position(1:2);
end

claimedActorIDs = [];   % DIAGNOSTIC (Day 4): actors already matched to a
                        % cluster this frame -- see sihLidarDiag.m header

for L = uniqueLabels(:)'
    mask        = (labels == L);
    clusterSize = nnz(mask);
    clusterPts  = pts(mask, :);
    centroid    = mean(clusterPts, 1);   % [x y z], assumed ego-frame --
    % computed up front (v5): every diagnostic path below, including
    % clusters the size filter is about to drop, needs the centroid.

    if clusterSize < minPoints || clusterSize > maxPoints
        if clusterSize < minPoints
            % Day 4 (v5): does a real actor sit within the attribution
            % gate of a cluster minPoints=5 is about to drop as noise?
            % Same WrongfulReject-style check as the ground-height filter
            % below -- answers whether minPoints=5 is wrongly dropping
            % real small detections, or whether this is genuinely noise.
            actorID = sihLidarNearestActor(centroid, poses, truePos);
            if actorID > 0 && isKey(classOf, actorID)
                classNum = double(classOf(actorID));
            elseif actorID > 0
                classNum = 0;   % AgentClass.Unknown
            else
                classNum = NaN;
            end
            sihLidarDiag('record', 'FilteredTooSmall', clusterSize, centroid(3), classNum);
        else
            sihLidarDiag('record', 'FilteredTooBig', clusterSize, centroid(3));
        end
        continue
    end

    zStd = std(clusterPts(:,3));
    isGroundLike = centroid(3) < cfg.Lidar.GroundZMax && zStd < cfg.Lidar.GroundZStdMax;

    [actorID, dmin] = sihLidarNearestActor(centroid, poses, truePos);

    % Day 4: reject ground-plane fragments that a point-count filter alone
    % can't catch -- see cfg.Lidar.GroundZ* header in sihConfig.m. The
    % actor-gate check above now runs FIRST purely so the diagnostic below
    % can tell a "clean" ground rejection (no real actor anywhere nearby)
    % apart from a "wrongful" one (an actor WAS within the 2.0m gate, so
    % this cluster would have been Matched/OverSegmented without the
    % height filter) -- the actual behavior (continue, skip attribution)
    % is unchanged either way; this is measurement only.
    if isGroundLike
        if actorID > 0
            if isKey(classOf, actorID)
                classNum = double(classOf(actorID));
            else
                classNum = 0;   % AgentClass.Unknown
            end
            sihLidarDiag('record', 'WrongfulReject', clusterSize, centroid(3), classNum);
        else
            sihLidarDiag('record', 'GroundRejected', clusterSize, centroid(3));
        end
        continue
    end

    % DIAGNOSTIC (Day 4) -- record only, does not affect actorID/reported
    % below or anything sihLidarClusterDetections.m returns. clusterSize
    % and centroid height (z) are passed through so ground-plane
    % fragments (small, low z) can be told apart from scattered noise.
    clusterZ = centroid(3);
    if actorID > 0
        if ismember(actorID, claimedActorIDs)
            sihLidarDiag('record', 'OverSegmented', clusterSize, clusterZ);
        else
            sihLidarDiag('record', 'Matched', clusterSize, clusterZ);
            % (v5) empirical centroid-to-truth position error for this
            % Matched cluster -- dmin IS that distance, already computed
            % by the actor-gate helper above (the nearest truePos point,
            % which for a Matched cluster is by definition the matched
            % actor). Feeds the R noise-model question in sihLidarDiag summary.
            sihLidarDiag('recordPosError', dmin);
            claimedActorIDs(end+1) = actorID; %#ok<AGROW>
        end
    else
        sihLidarDiag('record', 'Unattributed', clusterSize, clusterZ);
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

% ------------------------------------------------------------------------
function [actorID, dmin] = sihLidarNearestActor(centroid, poses, truePos)
%SIHLIDARNEARESTACTOR  Nearest real actor to a cluster centroid, gated at 2.0m.
%   Shared actor-attribution check (v5): needed a third time -- ground-
%   rejection attribution, class-confusion attribution, and the Day 4
%   FilteredTooSmall diagnostic -- so it's a helper now instead of being
%   duplicated inline again. dmin is the raw nearest-actor distance
%   regardless of whether the 2.0m gate passed (callers that need the
%   empirical position error, e.g. the Matched diagnostic, reuse it
%   directly rather than recomputing).
actorID = -1;
dmin    = Inf;
if ~isempty(poses)
    d = hypot(truePos(:,1) - centroid(1), truePos(:,2) - centroid(2));
    [dmin, idx] = min(d);
    if dmin < 2.0   % gate: don't attribute to a wildly distant actor
        actorID = poses(idx).ActorID;
    end
end
end
