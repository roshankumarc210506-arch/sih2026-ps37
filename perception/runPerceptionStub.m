function results = runPerceptionStub(varargin)
%RUNPERCEPTIONSTUB  M1 Day-1 deliverable: end-to-end perception pipeline.
%
%   results = runPerceptionStub();
%   results = runPerceptionStub('Visualize', false);
%   results = runPerceptionStub('Export',    true);   % writes .mat for M6
%
%   Pipeline:
%     drivingScenario -> targetPoses (ground truth)
%                     -> sihDummyDetections   (camera + radar + LiDAR stubs)
%                     -> trackerJPDA/GNN      (geometric fusion)
%                     -> sihClassVoter        (semantic fusion)
%                     -> sihTracksToContract  (locked bus format)
%
%   Prints a validation summary against ground truth so you have real
%   numbers for the Phase-1 checkpoint and the report, not just "it runs".

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'buses'));  % AgentClass, sihCreateBuses moved here

p = inputParser;
addParameter(p, 'Visualize', true,  @islogical);
addParameter(p, 'Export',    false, @islogical);
parse(p, varargin{:});
opts = p.Results;

cfg = sihConfig();
rng(cfg.RandomSeed);

sihCreateBuses(cfg);
[scenario, egoVehicle, classOf] = sihBuildScenario(cfg);
[tracker, trackerName]          = sihCreateTracker(cfg);
sihClassVoter('reset');
sihLidarDiag('reset');   % DIAGNOSTIC (Day 4) -- see sihLidarDiag.m header

[visionSensor, radarSensor] = sihCreateRealSensors(scenario, egoVehicle, cfg);
lidarSensor = sihCreateLidar(scenario, egoVehicle, cfg);

% ---- actor names, for the per-actor diagnostic table (diagnostics only) ----
nameMap = containers.Map('KeyType','double','ValueType','char');
for kA = 1:numel(scenario.Actors)
    act = scenario.Actors(kA);
    if isKey(classOf, act.ActorID)   % excludes ego, which has no classOf entry
        nameMap(act.ActorID) = act.Name;
    end
end

% ---------------- logging ----------------
log = struct('time', {}, 'tracks', {}, 'num_tracks', {}, 'ego', {});
val = struct('posErr', [], 'classOK', [], 'classTotal', 0, ...
             'nTruth', [], 'nTrack', [], 'falseTracks', [], 'missedObjects', [], ...
             'perClassTotal', zeros(1,7), 'perClassCorrect', zeros(1,7), ...
             'classPresent', zeros(1,7), 'classMatched', zeros(1,7), 'classMissed', zeros(1,7), ...
             'missClass', [], 'missRange', [], 'missAzimuth', [], 'missTime', [], ...
             'rangeBinMiss', zeros(1,11), 'rangeBinTotal', zeros(1,11), ...  % 10x10m bins (0-100) + 1 overflow (>100m)
             'azBinMiss', zeros(1,3), 'azBinTotal', zeros(1,3), ...
             'actorPresent', containers.Map('KeyType','double','ValueType','double'), ...
             'actorTracked', containers.Map('KeyType','double','ValueType','double'), ...
             'posErrROI', [], 'classOKROI', [], 'classTotalROI', 0, ...
             'missedObjectsROI', [], 'falseTracksROI', [], ...
             'perClassTotalROI', zeros(1,7), 'perClassCorrectROI', zeros(1,7), ...
             'falseBySensor', struct('CameraOnly',0,'RadarOnly',0,'LidarOnly',0,'Multi',0,'Unknown',0), ...
             'falseNearMatched', [], 'falseIsolated', 0, 'falseNearMatchedGate', 3.0, ...
             'dedupRemovedTotal', 0);
             % dedupRemovedTotal: counts confirmed tracks removed by the
             % Day 4 post-hoc dedup filter (see the main loop, right after
             % sihTracksToContract) -- lightweight visibility into whether
             % the filter is actually firing, not a correctness check.
             % falseNearMatchedGate = 3.0m -- DIAGNOSTIC (Day 4). Generous
             % on purpose: two tracks of the SAME real object should sit
             % far closer than this; genuinely distinct objects at normal
             % road spacing should sit farther. Not yet verified against
             % actual inter-object spacing in this scenario -- read the
             % printed distance stats before trusting this gate, same
             % caution as every other REASONED-DEFAULT constant this week.
             % Stored here (not a bare local in localValidate) so both the
             % check itself and this print block share ONE value -- see
             % this file's own Day 3 "five confirmed instances" lesson
             % about duplicated constants.
             % falseBySensor: DIAGNOSTIC ONLY, added Day 4 to triage the
             % post-MaxTracks-bump false-track jump before retuning
             % cfg.Tracker. Counts false-track-FRAMES (same convention as
             % falseTracksMean), classified by which sensor(s) appear in
             % that track's ObjectAttributes history -- see localValidate.

if opts.Visualize
    [bep, plotters] = localSetupPlot(cfg);
end

% ---------------- main loop ----------------
while advance(scenario)
    t     = scenario.SimulationTime;
    poses = targetPoses(egoVehicle);            % ego-frame ground truth

    dets = sihRealDetections(visionSensor, radarSensor, lidarSensor, egoVehicle, poses, t, cfg, classOf);

    if isempty(dets)
        tracks = predictTracksToTime(tracker, 'confirmed', t);
    else
        for s = 1:numel(cfg.Sensor)
            sd = dets(cellfun(@(d) d.SensorIndex == s, dets));
            if ~isempty(sd)
                ts = t + (s-1)*1e-4;
                for k = 1:numel(sd)
                    sd{k}.Time = ts;   % match the step-call time: OOSMHandling checks each detection's own Time against it
                end
                tracks = tracker(sd, ts);
            end
        end
    end

    classes = sihClassVoter('update', tracks, dets, cfg);
    [trackArray, n] = sihTracksToContract(tracks, classes, cfg);

    % ---- Day 4: post-hoc dedup of confirmed tracks (see sihConfig.m
    % cfg.Tracker.DedupCapDistance for the full rationale). SAME
    % distance+class check as the NearMatched diagnostic in localValidate
    % below -- promoted from measurement to an actual filter. Keeps
    % tracks/trackArray/n in lockstep, since localValidate relies on that
    % positional correspondence.
    %
    % REPORTING CAVEAT: dedup measurably raised classAcc (60.5%->63.4%
    % all-range). This is NOT the classifier getting better at telling
    % objects apart -- no classification logic changed here. It's a
    % denominator effect: duplicate tracks tend to be lower-confidence
    % (partial/asymmetric sensor support), so removing them removes
    % low-confidence entries from classAcc's denominator, same population
    % now scored differently rather than a genuinely improved population.
    % State it this way in the Day 8 report and to the team -- not as
    % "classification accuracy improved."
    if n > 1
        keep = true(n, 1);
        for i = 1:n
            if ~keep(i), continue; end
            for j = i+1:n
                if ~keep(j), continue; end
                d = hypot(trackArray(i).x - trackArray(j).x, trackArray(i).y - trackArray(j).y);
                compatibleClass = (trackArray(i).class == trackArray(j).class) || ...
                                   trackArray(i).class == 0 || trackArray(j).class == 0; % 0 = AgentClass.Unknown
                if d < cfg.Tracker.DedupCapDistance && compatibleClass
                    if trackArray(i).id <= trackArray(j).id
                        keep(j) = false;
                    else
                        keep(i) = false;
                        break   % i is now discarded -- stop comparing it to further j's
                    end
                end
            end
        end
        val.dedupRemovedTotal = val.dedupRemovedTotal + sum(~keep);

        % Day 4 BUGFIX (found via M2's coder.load failure -- her export
        % showed tracks arrays varying 2-16 elements/frame instead of a
        % fixed 40, traced back to here): trackArray(keep) applied a
        % SHORTER-than-40 logical mask directly to the full 40-slot
        % contract array, silently COLLAPSING it down to however many
        % survivors there were, destroying the fixed-size padding
        % structure sihTracksToContract.m guarantees. This never showed
        % up in any validation metric (RMSE/false-tracks/etc. don't care
        % about array size) -- only coder.load's strict codegen check
        % caught it. Fixed by restricting to the real (1:n) slots FIRST,
        % then rebuilding a full 40-slot array with survivors up front
        % and padding behind -- exactly the shape sihTracksToContract.m
        % itself produces.
        survivors  = trackArray(1:n);
        survivors  = survivors(keep);
        tracks     = tracks(keep);
        nSurvivors = sum(keep);

        if n < cfg.MaxTracks
            emptyTemplate = trackArray(cfg.MaxTracks);   % guaranteed a real
            % padding slot from sihTracksToContract.m, since n < MaxTracks
        else
            % Edge case: all 40 slots were real tracks, no padding exists
            % to copy from. Built inline -- must match
            % sihTracksToContract.m's sihEmptyTrack() EXACTLY (that
            % function is local/private to that file, not callable here).
            emptyTemplate = struct( ...
                'id',         uint32(0), ...
                'class',      AgentClass.Unknown, ...
                'x',          0, ...
                'y',          0, ...
                'heading',    0, ...
                'velocity',   0, ...
                'covariance', zeros(4,4), ...
                'valid',      false);
        end
        trackArray = repmat(emptyTemplate, cfg.MaxTracks, 1);
        trackArray(1:nSurvivors) = survivors;
        n = nSurvivors;
    end

    % ---- publish (this struct IS the contract) ----
    perception.tracks     = trackArray;
    perception.num_tracks = n;
    perception.timestamp  = t;

    ep = egoVehicle.Position;
    ego = struct('x', ep(1), 'y', ep(2), ...
                 'yaw', deg2rad(egoVehicle.Yaw), ...
                 'velocity', norm(egoVehicle.Velocity(1:2)), ...
                 'Timestamp', t);

    log(end+1) = struct('time', t, 'tracks', trackArray, ...
                        'num_tracks', n, 'ego', ego);          %#ok<AGROW>

    val = localValidate(val, trackArray, tracks, n, poses, classOf, t, cfg);

    if opts.Visualize
        localUpdatePlot(bep, plotters, dets, trackArray, n, poses, cfg);
    end
end

% ---------------- summary ----------------
results.trackerName = trackerName;
results.log         = log;
results.numFrames   = numel(log);

results.posRMSE          = sqrt(mean(val.posErr.^2, 'omitnan'));
results.posP95           = prctileLite(val.posErr, 95);
results.classAcc         = sum(val.classOK) / max(val.classTotal, 1);
results.meanTracks       = mean(val.nTrack);
results.meanTruth        = mean(val.nTruth);
results.trackYield       = results.meanTracks / max(results.meanTruth, eps);
results.falseTracksMean  = mean(val.falseTracks);
results.missedObjectsMean= mean(val.missedObjects);

results.posRMSEROI           = sqrt(mean(val.posErrROI.^2, 'omitnan'));
results.classAccROI          = sum(val.classOKROI) / max(val.classTotalROI, 1);
results.missedObjectsMeanROI = mean(val.missedObjectsROI);
results.falseTracksMeanROI   = mean(val.falseTracksROI);

fprintf('\n=============== M1 PERCEPTION — DAY 1 RESULT ===============\n');
fprintf('  Tracker used ............. %s\n',        results.trackerName);
fprintf('  Frames simulated ......... %d\n',        results.numFrames);
fprintf('  ---- ALL RANGES ----\n');
fprintf('  Position RMSE ............ %.2f m\n',    results.posRMSE);
fprintf('  Position error p95 ....... %.2f m\n',    results.posP95);
fprintf('  Classification accuracy .. %.1f %%\n',   100*results.classAcc);
fprintf('  Mean tracks / mean truth . %.2f / %.2f  (yield %.0f %%)\n', ...
        results.meanTracks, results.meanTruth, 100*results.trackYield);
fprintf('  False tracks / frame (mean)   . %.2f\n', results.falseTracksMean);
fprintf('  Missed objects / frame (mean) . %.2f\n',  results.missedObjectsMean);
fprintf('  Dedup: confirmed tracks removed (total / per frame) . %d / %.2f\n', ...
    val.dedupRemovedTotal, val.dedupRemovedTotal/max(results.numFrames,1));
fprintf('  False tracks by sensor (DIAGNOSTIC, false-track-frames, not unique tracks):\n');
fbs = val.falseBySensor;
fbsTotal = fbs.CameraOnly + fbs.RadarOnly + fbs.LidarOnly + fbs.Multi + fbs.Unknown;
fprintf('    Camera-only .. %6d (%.0f%%)\n', fbs.CameraOnly, 100*fbs.CameraOnly/max(fbsTotal,1));
fprintf('    Radar-only ... %6d (%.0f%%)\n', fbs.RadarOnly,  100*fbs.RadarOnly /max(fbsTotal,1));
fprintf('    LiDAR-only ... %6d (%.0f%%)\n', fbs.LidarOnly,  100*fbs.LidarOnly /max(fbsTotal,1));
fprintf('    Multi-sensor . %6d (%.0f%%)\n', fbs.Multi,      100*fbs.Multi    /max(fbsTotal,1));
fprintf('    Unknown ...... %6d (%.0f%%)\n', fbs.Unknown,    100*fbs.Unknown  /max(fbsTotal,1));
fnmTotal = numel(val.falseNearMatched) + val.falseIsolated;
fprintf('  False tracks: isolated ghost vs near an already-matched track (DIAGNOSTIC):\n');
fprintf('    Isolated ....... %6d (%.0f%%)  -- no matched track within %.1fm\n', ...
    val.falseIsolated, 100*val.falseIsolated/max(fnmTotal,1), val.falseNearMatchedGate);
if ~isempty(val.falseNearMatched)
    fprintf('    NearMatched .... %6d (%.0f%%)  -- candidate duplicate/split of a real\n', ...
        numel(val.falseNearMatched), 100*numel(val.falseNearMatched)/max(fnmTotal,1));
    fprintf('                       object; median distance to nearest matched track: %.2fm\n', ...
        median(val.falseNearMatched));
else
    fprintf('    NearMatched ....      0 (0%%)\n');
end
lidarDiag = sihLidarDiag('summary');
lidarDiagTotal = lidarDiag.Matched.Count + lidarDiag.OverSegmented.Count + lidarDiag.Unattributed.Count ...
                 + lidarDiag.GroundRejected.Count + lidarDiag.WrongfulReject.Count;
fprintf('  LiDAR cluster attribution (DIAGNOSTIC, per-cluster over the full run):\n');
fprintf('    %-14s %8s %8s | %10s %10s %10s %10s\n', 'Bucket', 'Count', '%%', 'MedSize', 'MaxSize', 'MedZ(m)', 'ZRange(m)');
lidarBuckets = {'GroundRejected', 'WrongfulReject', 'Matched', 'OverSegmented', 'Unattributed'};
for b = 1:numel(lidarBuckets)
    bk = lidarDiag.(lidarBuckets{b});
    pct = 100 * bk.Count / max(lidarDiagTotal, 1);
    if bk.Count > 0
        fprintf('    %-14s %8d %7.0f%% | %10.1f %10d %10.2f %10.2f\n', ...
            lidarBuckets{b}, bk.Count, pct, median(bk.Sizes), max(bk.Sizes), ...
            median(bk.Z), max(bk.Z) - min(bk.Z));   % manual range -- avoids
            % depending on Statistics Toolbox's range(), same reasoning as
            % prctileLite() elsewhere in this file.
    else
        fprintf('    %-14s %8d %7.0f%% | %10s %10s %10s %10s\n', lidarBuckets{b}, 0, 0, '-', '-', '-', '-');
    end
end
fprintf('    (GroundRejected = ground filter fired, no actor was within the 2.0m gate\n');
fprintf('     anyway -- a clean rejection. WrongfulReject = ground filter fired but an\n');
fprintf('     actor WAS within the gate -- this cluster would have been Matched or\n');
fprintf('     OverSegmented without the height filter. Its class breakdown is below.)\n');
wr = lidarDiag.WrongfulReject;
if wr.Count > 0
    classNames = {'Unknown','Car','TwoWheeler','AutoRickshaw','PushCart','Pedestrian','Animal'};
    fprintf('    WrongfulReject by class (this is the height filter''s real tradeoff cost):\n');
    for c = 0:6
        n = sum(wr.Classes == c);
        if n > 0
            fprintf('      %-12s %6d (%.0f%% of all WrongfulReject)\n', classNames{c+1}, n, 100*n/wr.Count);
        end
    end
end
rawCounts = lidarDiag.RawClusterCounts;
fprintf('  Raw LiDAR clusters/frame BEFORE size filter (DIAGNOSTIC): mean %.2f, median %.1f, min %d, max %d\n', ...
    mean(rawCounts), median(rawCounts), min(rawCounts), max(rawCounts));
fprintf('  ---- PLANNING ROI (%gm, forward) ----\n', cfg.ROI.MaxRange);
fprintf('  Position RMSE ............ %.2f m\n',    results.posRMSEROI);
fprintf('  Classification accuracy .. %.1f %%\n',   100*results.classAccROI);
fprintf('  Missed objects / frame (mean) . %.2f\n',  results.missedObjectsMeanROI);
fprintf('  False tracks / frame (mean)   . %.2f\n', results.falseTracksMeanROI);
fprintf('------------------------------------------------------------\n');
fprintf('  Per-class accuracy (matched tracks only):\n');
fprintf('    %-14s %8s %8s %8s | %8s %8s %8s\n', 'Class', 'Correct', 'Total', 'Acc %', 'ROI-Cor', 'ROI-Tot', 'ROI-Acc%');
for v = 1:6
    cls     = AgentClass(v);
    tot     = val.perClassTotal(v+1);
    corr    = val.perClassCorrect(v+1);
    pct     = 100 * corr / max(tot, 1);
    totROI  = val.perClassTotalROI(v+1);
    corrROI = val.perClassCorrectROI(v+1);
    pctROI  = 100 * corrROI / max(totROI, 1);
    fprintf('    %-14s %8d %8d %8.1f | %8d %8d %8.1f\n', char(cls), corr, tot, pct, corrROI, totROI, pctROI);
end

fprintf('------------------------------------------------------------\n');
fprintf('  MISSES BY CLASS:\n');
fprintf('    %-14s %8s %8s %8s %8s\n', 'Class', 'Present', 'Matched', 'Missed', 'Miss %');
for v = 1:6
    cls   = AgentClass(v);
    pres  = val.classPresent(v+1);
    matc  = val.classMatched(v+1);
    miss  = val.classMissed(v+1);
    mrate = 100 * miss / max(pres, 1);
    fprintf('    %-14s %8d %8d %8d %8.1f\n', char(cls), pres, matc, miss, mrate);
end

fprintf('------------------------------------------------------------\n');
fprintf('  MISSES BY RANGE (10 m bins from ego):\n');
fprintf('    %-14s %8s %8s %8s\n', 'Range', 'Misses', 'Total', 'Miss %');
for b = 1:11
    if b <= 10
        label = sprintf('%d-%dm', (b-1)*10, b*10);
    else
        label = '>100m';
    end
    miss  = val.rangeBinMiss(b);
    tot   = val.rangeBinTotal(b);
    mrate = 100 * miss / max(tot, 1);
    fprintf('    %-14s %8d %8d %8.1f\n', label, miss, tot, mrate);
end

fprintf('------------------------------------------------------------\n');
fprintf('  MISSES BY AZIMUTH:\n');
fprintf('    %-14s %8s %8s %8s\n', 'Azimuth', 'Misses', 'Total', 'Miss %');
azLabels = {'<=20 deg', '20-30 deg', '>30 deg'};
for b = 1:3
    miss  = val.azBinMiss(b);
    tot   = val.azBinTotal(b);
    mrate = 100 * miss / max(tot, 1);
    fprintf('    %-14s %8d %8d %8.1f\n', azLabels{b}, miss, tot, mrate);
end

fprintf('------------------------------------------------------------\n');
fprintf('  PER-ACTOR TRACKING:\n');
fprintf('    %-14s %-14s %8s %8s %8s\n', 'Actor', 'Class', 'Present', 'Tracked', '% Tracked');
actorIDs = sort(cell2mat(keys(nameMap)));
for ai = 1:numel(actorIDs)
    aid  = actorIDs(ai);
    name = nameMap(aid);
    cls  = classOf(aid);
    pres = val.actorPresent(aid);
    trk  = val.actorTracked(aid);
    pct  = 100 * trk / max(pres, 1);
    fprintf('    %-14s %-14s %8d %8d %8.1f\n', name, char(cls), pres, trk, pct);
end
fprintf('============================================================\n');
fprintf('Bus format verified: {id, class, x, y, heading, velocity, covariance}\n');
fprintf('Fixed array of %d + num_tracks + timestamp. Ego pose on side bus.\n\n', cfg.MaxTracks);

if opts.Export
    sihExportForSimulink(log, cfg);
end
end

% ========================================================================
function val = localValidate(val, trackArray, tracks, n, poses, classOf, t, cfg)
%LOCALVALIDATE  Greedy one-to-one match of tracks to ground truth.
%   Repeatedly takes the globally smallest track-truth distance, assigns
%   that pair, and removes both from contention, until the smallest
%   remaining distance exceeds the gate. Prevents multiple tracks from
%   all being scored against the same truth object (which happens with
%   plain nearest-neighbour matching whenever tracks outnumber truths).
%
%   Also logs, for every unmatched truth object (a miss): its AgentClass,
%   range and azimuth from ego, and the frame time — diagnostics only,
%   doesn't affect matching or any existing metric.
%
%   Alongside the all-range metrics, accumulates a second, ROI-restricted
%   set (cfg.ROI.MaxRange / cfg.ROI.MaxAbsAzim): matched pairs, false
%   tracks, and missed objects are additionally counted only when the
%   relevant truth (or, for false tracks, the track itself) falls inside
%   the planning region. Purely additive — every existing accumulator and
%   the matching logic itself are untouched.
%
%   DAY 4 ADDITION — falseBySensor: `tracks` (the raw tracker output, not
%   trackArray) is now passed in solely so every unmatched (false) track
%   can be classified by which sensor(s) appear in its ObjectAttributes
%   history (see sihRealDetections.m / sihLidarClusterDetections.m — every
%   detection tags SensorName, and the tracker concatenates that history
%   per track). trackArray(i) and tracks(i) are the same track in the same
%   order (sihTracksToContract.m loops i=1:numTracks over both in lockstep),
%   so index i is reused directly. Diagnostic only — does not affect any
%   existing metric or the matching logic above it.
gate = 3.0;   % m
n = double(n);
nTruth = numel(poses);
val.nTruth(end+1) = nTruth;
val.nTrack(end+1) = n;

% ---- per-truth position/range/azimuth/class, and presence tallies ----
% (computed regardless of match outcome, so "present" counts are correct
% even on frames with zero tracks)
tp    = zeros(nTruth, 2);
rng_  = zeros(nTruth, 1);
az_   = zeros(nTruth, 1);
truthClassArr = repmat(AgentClass.Unknown, nTruth, 1);
truthInROI    = false(nTruth, 1);

for k = 1:nTruth
    pk = poses(k).Position;
    tp(k,:) = pk(1:2);
    rng_(k) = hypot(pk(1), pk(2));
    az_(k)  = atan2d(pk(2), pk(1));
    truthInROI(k) = localInROI(rng_(k), az_(k), cfg);

    aid = poses(k).ActorID;
    truthClassArr(k) = classOf(aid);

    cIdx = double(truthClassArr(k)) + 1;
    val.classPresent(cIdx) = val.classPresent(cIdx) + 1;

    rb = localRangeBin(rng_(k));
    val.rangeBinTotal(rb) = val.rangeBinTotal(rb) + 1;

    ab = localAzBin(az_(k));
    val.azBinTotal(ab) = val.azBinTotal(ab) + 1;

    if isKey(val.actorPresent, aid)
        val.actorPresent(aid) = val.actorPresent(aid) + 1;
    else
        val.actorPresent(aid) = 1;
    end
    if ~isKey(val.actorTracked, aid)
        val.actorTracked(aid) = 0;
    end
end

if n == 0 || nTruth == 0
    val.falseTracks(end+1)   = n;
    val.missedObjects(end+1) = nTruth;
    val.falseTracksROI(end+1)   = 0;
    val.missedObjectsROI(end+1) = sum(truthInROI);
    for k = 1:nTruth
        val = localLogMiss(val, truthClassArr(k), rng_(k), az_(k), t);
    end
    for i = 1:n   % nTruth==0 case: every track this frame is false, and
                  % trivially ISOLATED (nothing could have matched this
                  % frame for it to be "near")
        val = localTagFalseSensor(val, tracks(i));
        val.falseIsolated = val.falseIsolated + 1;
    end
    return
end

cost = zeros(n, nTruth);
trackInROI = false(n, 1);
for i = 1:n
    cost(i,:) = hypot(tp(:,1) - trackArray(i).x, tp(:,2) - trackArray(i).y)';
    trackRange = hypot(trackArray(i).x, trackArray(i).y);
    trackAz    = atan2d(trackArray(i).y, trackArray(i).x);
    trackInROI(i) = localInROI(trackRange, trackAz, cfg);
end

matchedTrack = false(n, 1);
matchedTruth = false(nTruth, 1);

while true
    C = cost;
    C(matchedTrack, :) = Inf;
    C(:, matchedTruth) = Inf;
    [dmin, idx] = min(C(:));
    if isinf(dmin) || dmin > gate, break; end

    [i, k] = ind2sub(size(C), idx);
    matchedTrack(i) = true;
    matchedTruth(k) = true;

    truthClass = truthClassArr(k);
    isOK = double(trackArray(i).class == truthClass);

    val.posErr(end+1)  = dmin;
    val.classTotal     = val.classTotal + 1;
    val.classOK(end+1) = isOK;

    cIdx = double(truthClass) + 1;   % AgentClass 0..6 -> index 1..7
    val.perClassTotal(cIdx)   = val.perClassTotal(cIdx) + 1;
    val.perClassCorrect(cIdx) = val.perClassCorrect(cIdx) + isOK;

    if truthInROI(k)
        val.posErrROI(end+1)  = dmin;
        val.classTotalROI     = val.classTotalROI + 1;
        val.classOKROI(end+1) = isOK;
        val.perClassTotalROI(cIdx)   = val.perClassTotalROI(cIdx) + 1;
        val.perClassCorrectROI(cIdx) = val.perClassCorrectROI(cIdx) + isOK;
    end
end

val.falseTracks(end+1)   = sum(~matchedTrack);
val.missedObjects(end+1) = sum(~matchedTruth);
val.falseTracksROI(end+1)   = sum(~matchedTrack & trackInROI);
val.missedObjectsROI(end+1) = sum(~matchedTruth & truthInROI);

for i = 1:n
    if ~matchedTrack(i)
        val = localTagFalseSensor(val, tracks(i));
        if any(matchedTrack)
            dNear = min(hypot([trackArray(matchedTrack).x] - trackArray(i).x, ...
                               [trackArray(matchedTrack).y] - trackArray(i).y));
        else
            dNear = Inf;   % nothing matched this frame at all -- can't be "near" anything
        end
        if dNear < val.falseNearMatchedGate
            val.falseNearMatched(end+1) = dNear; %#ok<AGROW>
        else
            val.falseIsolated = val.falseIsolated + 1;
        end
    end
end

for k = 1:nTruth
    aid = poses(k).ActorID;
    if matchedTruth(k)
        val.classMatched(double(truthClassArr(k))+1) = val.classMatched(double(truthClassArr(k))+1) + 1;
        val.actorTracked(aid) = val.actorTracked(aid) + 1;
    else
        val = localLogMiss(val, truthClassArr(k), rng_(k), az_(k), t);
    end
end
end

% ========================================================================
function val = localTagFalseSensor(val, track)
%LOCALTAGFALSESENSOR  DIAGNOSTIC (Day 4). Classify one false track by
%   which sensor(s) appear in its ObjectAttributes history.
%   NOT verified against a live run yet — SensorName is set correctly at
%   every detection source (checked in sihRealDetections.m and
%   sihLidarClusterDetections.m), and per the Day 3 concatenation-crash
%   fix we know ObjectAttributes accumulates across a track's life rather
%   than holding only the latest entry, but the exact struct-array shape
%   MATLAB hands back here hasn't been printed and inspected firsthand.
%   If this errors or every track comes back 'Unknown', print
%   tracks(1).ObjectAttributes directly and fix the field access below —
%   inspect, don't guess a second time (same rule as sihRecoverActorID).
names = {};
attrs = track.ObjectAttributes;
if ~isempty(attrs)
    for a = 1:numel(attrs)
        if isfield(attrs(a), 'SensorName')
            names{end+1} = attrs(a).SensorName; %#ok<AGROW>
        end
    end
end
uNames = unique(names);

if isempty(uNames)
    key = 'Unknown';
elseif isscalar(uNames)
    switch uNames{1}
        case 'Camera', key = 'CameraOnly';
        case 'Radar',  key = 'RadarOnly';
        case 'LiDAR',  key = 'LidarOnly';
        otherwise,     key = 'Unknown';
    end
else
    key = 'Multi';
end
val.falseBySensor.(key) = val.falseBySensor.(key) + 1;
end

% ========================================================================
function tf = localInROI(rangeVal, azVal, cfg)
%LOCALINROI  True if a range/azimuth pair falls inside the planning ROI.
tf = (rangeVal <= cfg.ROI.MaxRange) && (abs(azVal) <= cfg.ROI.MaxAbsAzim);
end

% ========================================================================
function val = localLogMiss(val, cls, rangeVal, azVal, t)
%LOCALLOGMISS  Record one missed-truth event and bucket it for the summary.
val.missClass(end+1)   = double(cls);
val.missRange(end+1)   = rangeVal;
val.missAzimuth(end+1) = azVal;
val.missTime(end+1)    = t;

cIdx = double(cls) + 1;
val.classMissed(cIdx) = val.classMissed(cIdx) + 1;

rb = localRangeBin(rangeVal);
val.rangeBinMiss(rb) = val.rangeBinMiss(rb) + 1;

ab = localAzBin(azVal);
val.azBinMiss(ab) = val.azBinMiss(ab) + 1;
end

% ========================================================================
function b = localRangeBin(r)
%LOCALRANGEBIN  10 m bins from 0-100 m, plus an overflow bin for >100 m.
if r >= 100
    b = 11;
else
    b = max(1, min(10, floor(r/10) + 1));
end
end

% ========================================================================
function b = localAzBin(az)
%LOCALAZBIN  |az|<=20 deg -> 1, 20-30 deg -> 2, >30 deg -> 3.
a = abs(az);
if a <= 20
    b = 1;
elseif a <= 30
    b = 2;
else
    b = 3;
end
end

% ========================================================================
function [bep, plotters] = localSetupPlot(cfg)
figure('Name', 'M1 Perception — sensor fusion stub', 'Position', [80 80 900 620]);
ax  = axes;
bep = birdsEyePlot('Parent', ax, 'XLim', [-20 90], 'YLim', [-30 30]);

for s = 1:numel(cfg.Sensor)
    S = cfg.Sensor(s);
    plotters.cov(s) = coverageAreaPlotter(bep, ...
        'DisplayName', S.Name, 'FaceColor', localSensorColor(s));
    plotCoverageArea(plotters.cov(s), [0 0], S.MaxRange, 0, min(S.FoV,359));
    plotters.det(s) = detectionPlotter(bep, ...
        'DisplayName', [S.Name ' det'], 'MarkerEdgeColor', localSensorColor(s), ...
        'Marker', '+');
end

plotters.truth = trackPlotter(bep, 'DisplayName', 'ground truth', ...
    'MarkerEdgeColor', [0.5 0.5 0.5]);
plotters.trk = trackPlotter(bep, 'DisplayName', 'fused tracks', ...
    'MarkerEdgeColor', 'k', 'HistoryDepth', 12);
title(ax, 'Perception stub — camera / radar / LiDAR fused');
end

% ========================================================================
function localUpdatePlot(~, plotters, dets, trackArray, n, poses, cfg)
for s = 1:numel(cfg.Sensor)
    pos = zeros(0,3);
    for d = 1:numel(dets)
        if dets{d}.SensorIndex == s
            pos(end+1,:) = dets{d}.Measurement(:)';    %#ok<AGROW>
        end
    end
    plotDetection(plotters.det(s), pos);
end

% ground truth
gp = zeros(numel(poses), 3);
for k = 1:numel(poses)
    gp(k,:) = poses(k).Position;
end
plotTrack(plotters.truth, gp);

% fused tracks, labelled with the fused class
tp  = zeros(double(n), 3);
lbl = cell(double(n), 1);
for i = 1:double(n)
    tp(i,:)  = [trackArray(i).x, trackArray(i).y, 0];
    lbl{i}   = sprintf('%d:%s', trackArray(i).id, char(trackArray(i).class));
end
plotTrack(plotters.trk, tp, lbl);
drawnow limitrate
end

% ========================================================================
function c = localSensorColor(s)
cols = [0.85 0.33 0.10;    % camera - orange
        0.00 0.45 0.74;    % radar  - blue
        0.47 0.67 0.19];   % lidar  - green
c = cols(min(s,3), :);
end

% ========================================================================
function v = prctileLite(x, p)
%PRCTILELITE  Percentile without needing Statistics Toolbox.
x = sort(x(~isnan(x)));
if isempty(x), v = NaN; return; end
idx = max(1, min(numel(x), ceil(p/100 * numel(x))));
v   = x(idx);
end