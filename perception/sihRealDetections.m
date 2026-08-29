function dets = sihRealDetections(visionSensor, radarSensor, lidarSensor, egoVehicle, poses, time, cfg, classOf)
%SIHREALDETECTIONS  Day 3 detections: real camera + radar + LiDAR.
%
%   dets = sihRealDetections(visionSensor, radarSensor, lidarSensor, egoVehicle, poses, time, cfg, classOf)
%
%   Same output contract as Day 1's sihDummyDetections: a cell array of
%   objectDetection with SensorIndex set (1/2/3), ready for the tracker
%   exactly as before. Nothing downstream — tracker, class voter,
%   contract conversion, validation — needs to change.
%
%   CLASS HANDLING — a deliberate choice, not an oversight:
%   visionDetectionGenerator/drivingRadarDataGenerator report class from
%   ActorProfiles, which only carries MATLAB's own restricted vocabulary
%   (via the sihScenarioClassID mapping from Day 1's ClassID fix), and
%   they don't model semantic misclassification — only geometric
%   detection effects (range, occlusion, noise). Rather than reverse-
%   translating their passthrough ClassID through that lossy mapping, we
%   keep applying OUR OWN class-confusion model (cfg.Sensor(s).ClassAcc +
%   cfg.ConfusionPairs) on top of the real geometric output, same as the
%   Day 1 stub. Keeps the one part we actually want to tune — how often
%   a camera mistakes a pushcart for an animal — under our own control.
%
%   ACTORID RECOVERY — flagged for verification on first run:
%   to look up classOf(ActorID) per detection we need to know which input
%   target it came from. Expected field: dets{k}.ObjectAttributes{1}.TargetIndex
%   (index into `poses`), giving poses(TargetIndex).ActorID. If this
%   throws or returns nonsense, PRINT dets{1} IN FULL and fix the field
%   name in sihRecoverActorID below — inspect, don't guess a second time.
%
%   KNOWN BEHAVIOR (verified Day 2, not a bug): real vision/radar report
%   detections from an object's near visible surface, not its centroid.
%   Offset scales with object size — confirmed via 3 lines of evidence:
%   (1) MathWorks' own radar tutorial documents the same effect for
%       extended targets, (2) delta direction tracks each target's
%       individual bearing to ego across a 24 deg spread rather than
%       staying fixed (rules out a mounting-offset frame bug),
%       (3) delta magnitude scales with object length (3.5m on a 4.5m
%       car vs 0.4-1.2m on smaller actors).
%   This is why Day 2's ROI RMSE (0.26m) is HIGHER than Day 1's stub
%   (0.18m) — Day 2 is more physically honest, not worse. Flagged to
%   M3 since it affects costmap safety-margin inflation for large actors.

dets = {};

% ---------------- Camera (real) ----------------
[visDets, numVis] = visionSensor(poses, time);
for k = 1:numVis
    d = visDets{k};
    actorID = sihRecoverActorID(d, poses);
    d = sihApplyClassConfusion(d, actorID, classOf, cfg, 1);
    dets{end+1} = d; %#ok<AGROW>
end

% ---------------- Radar (real) ----------------
[radDets, numRad] = radarSensor(poses, time);
for k = 1:numRad
    d = radDets{k};
    actorID = sihRecoverActorID(d, poses);
    d = sihApplyClassConfusion(d, actorID, classOf, cfg, 2);
    dets{end+1} = d; %#ok<AGROW>
end

% ---------------- LiDAR (real, Day 3) ----------------
lidarDets = sihLidarClusterDetections(lidarSensor, egoVehicle, poses, time, cfg, classOf);
dets = [dets(:); lidarDets(:)];   % dets grows as a 1xN row cell via {end+1}=...; force columns so vertcat doesn't require equal row-lengths

dets = dets(:);
end

% ------------------------------------------------------------------------
function actorID = sihRecoverActorID(det, poses)
%SIHRECOVERACTORID  Trace a generated detection back to its source actor.
%   TargetIndex is the actor's real ActorID (checked directly against
%   MATLAB's docs: visionDetectionGenerator/drivingRadarDataGenerator
%   report ObjectAttributes.TargetIndex as the ActorID from the
%   Profiles/ActorProfiles struct passed at construction, NOT a position
%   in `poses`). poses excludes ego and is repositioned starting at 1, so
%   poses(idx) silently returns the WRONG actor for every idx that happens
%   to still be in-bounds (only out-of-range idx values, like ego-excluded
%   gaps or the -1 false-alarm sentinel, used to throw). Match by ActorID
%   value instead of treating TargetIndex as a position.
try
    idx     = det.ObjectAttributes{1}.TargetIndex;
    match   = find([poses.ActorID] == idx, 1);
    actorID = poses(match).ActorID;   % errors (caught below) if match is empty
catch
    % No poses entry has this ActorID: either a real false alarm
    % (TargetIndex == -1, MATLAB's own "no source actor" sentinel) or an
    % actor not currently in `poses`. Surface it loudly rather than
    % silently misclassifying every single detection.
    actorID = -1;
    warning('sihRealDetections:actorIDRecoveryFailed', ...
        ['Could not recover source ActorID from a real detection. ' ...
         'Inspect dets{1}.ObjectAttributes structure and fix ' ...
         'sihRecoverActorID to match the real field name.']);
end
end

% ------------------------------------------------------------------------
function d = sihApplyClassConfusion(d, actorID, classOf, cfg, sensorIdx)
if actorID > 0 && isKey(classOf, actorID)
    trueClass = classOf(actorID);
    if rand < cfg.Sensor(sensorIdx).ClassAcc
        reported = trueClass;
    else
        reported = sihConfuseClass(trueClass, cfg);
    end
else
    reported = AgentClass.Unknown;   % false alarm / unmatched
end
d.ObjectClassID    = double(reported);
d.ObjectAttributes = {struct('TruthActorID', actorID, ...
                              'TruthClass',   double(reported), ...
                              'SensorName',   cfg.Sensor(sensorIdx).Name)};
end

% sihConfuseClassPublic moved to its own shared file (perception/sihConfuseClass.m)
% -- was duplicated identically here and in sihDummyDetections.m; LiDAR
% clustering needed it too, making three call sites, worth one definition.

% sihStubLidarDetections deleted Day 3 -- superseded by real LiDAR
% (sihCreateLidar + sihLidarClusterDetections). Confirmed no remaining
% call sites (grep across perception/ and buses/) before removing it.
