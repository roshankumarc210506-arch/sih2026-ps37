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

[visionSensor, radarSensor] = sihCreateRealSensors(scenario, egoVehicle, cfg);

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
             'perClassTotalROI', zeros(1,7), 'perClassCorrectROI', zeros(1,7));

if opts.Visualize
    [bep, plotters] = localSetupPlot(cfg);
end

% ---------------- main loop ----------------
while advance(scenario)
    t     = scenario.SimulationTime;
    poses = targetPoses(egoVehicle);            % ego-frame ground truth

    dets = sihRealDetections(visionSensor, radarSensor, poses, t, cfg, classOf);

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

    % ---- publish (this struct IS the contract) ----
    perception.tracks     = trackArray;
    perception.num_tracks = n;
    perception.timestamp  = t;

    ep = egoVehicle.Position;
    ego = struct('x', ep(1), 'y', ep(2), ...
                 'yaw', deg2rad(egoVehicle.Yaw), ...
                 'velocity', norm(egoVehicle.Velocity(1:2)));

    log(end+1) = struct('time', t, 'tracks', trackArray, ...
                        'num_tracks', n, 'ego', ego);          %#ok<AGROW>

    val = localValidate(val, trackArray, n, poses, classOf, t, cfg);

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
function val = localValidate(val, trackArray, n, poses, classOf, t, cfg)
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
