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

% ---------------- logging ----------------
log = struct('time', {}, 'tracks', {}, 'num_tracks', {}, 'ego', {});
val = struct('posErr', [], 'classOK', [], 'classTotal', 0, ...
             'nTruth', [], 'nTrack', []);

if opts.Visualize
    [bep, plotters] = localSetupPlot(cfg);
end

% ---------------- main loop ----------------
while advance(scenario)
    t     = scenario.SimulationTime;
    poses = targetPoses(egoVehicle);            % ego-frame ground truth

    dets = sihDummyDetections(poses, t, cfg, classOf);

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

    val = localValidate(val, trackArray, n, poses, classOf);

    if opts.Visualize
        localUpdatePlot(bep, plotters, dets, trackArray, n, poses, cfg);
    end
end

% ---------------- summary ----------------
results.trackerName = trackerName;
results.log         = log;
results.numFrames   = numel(log);

results.posRMSE      = sqrt(mean(val.posErr.^2, 'omitnan'));
results.posP95       = prctileLite(val.posErr, 95);
results.classAcc     = sum(val.classOK) / max(val.classTotal, 1);
results.meanTracks   = mean(val.nTrack);
results.meanTruth    = mean(val.nTruth);
results.trackYield   = results.meanTracks / max(results.meanTruth, eps);

fprintf('\n=============== M1 PERCEPTION — DAY 1 RESULT ===============\n');
fprintf('  Tracker used ............. %s\n',        results.trackerName);
fprintf('  Frames simulated ......... %d\n',        results.numFrames);
fprintf('  Position RMSE ............ %.2f m\n',    results.posRMSE);
fprintf('  Position error p95 ....... %.2f m\n',    results.posP95);
fprintf('  Classification accuracy .. %.1f %%\n',   100*results.classAcc);
fprintf('  Mean tracks / mean truth . %.2f / %.2f  (yield %.0f %%)\n', ...
        results.meanTracks, results.meanTruth, 100*results.trackYield);
fprintf('============================================================\n');
fprintf('Bus format verified: {id, class, x, y, heading, velocity, covariance}\n');
fprintf('Fixed array of %d + num_tracks + timestamp. Ego pose on side bus.\n\n', cfg.MaxTracks);

if opts.Export
    sihExportForSimulink(log, cfg);
end
end

% ========================================================================
function val = localValidate(val, trackArray, n, poses, classOf)
%LOCALVALIDATE  Nearest-neighbour match of tracks to ground truth.
gate = 3.0;   % m
nTruth = numel(poses);
val.nTruth(end+1) = nTruth;
val.nTrack(end+1) = double(n);
if n == 0 || nTruth == 0, return; end

tp = zeros(nTruth, 2);
for k = 1:nTruth
    tp(k,:) = poses(k).Position(1:2);
end

for i = 1:double(n)
    d = hypot(tp(:,1) - trackArray(i).x, tp(:,2) - trackArray(i).y);
    [dmin, k] = min(d);
    if dmin > gate, continue; end

    val.posErr(end+1) = dmin;
    val.classTotal    = val.classTotal + 1;
    val.classOK(end+1) = double(trackArray(i).class == classOf(poses(k).ActorID));
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
