function cfg = sihConfig()
%SIHCONFIG  Single source of truth for M1 (Perception) parameters.
%   Everything tunable lives here so Phase 1 tuning is edit-one-file.
%
%   cfg = sihConfig();

% ---------- Timing ----------
cfg.SampleTime = 0.1;      % s   (10 Hz perception update)
cfg.StopTime   = 25;       % s

% ---------- Bus sizing (Simulink needs FIXED sizes) ----------
cfg.MaxTracks  = 40;       % fixed-length track array in the bus. Generous
                           % headroom, not a tight fit -- DenseMarket has
                           % 10 real actors and even moderate yield puts
                           % real demand well above the old 20. Data-loss
                           % fix, not metric-tuning: 222/248 frames were
                           % silently failing to initiate new tracks,
                           % meaning we didn't know whether real objects
                           % were being dropped some of those frames.
                           % FLAG: buses/sihDefineBuses.m (M6) and
                           % Prediction/*.m (M2) independently hardcode
                           % MAX_TRACKS=20 to "mirror" this value -- they
                           % will now be out of sync until updated there
                           % too. Not this file's job to fix; raise with
                           % M2/M6.
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

% ---------- LiDAR ground-plane rejection (Day 4 addition) ----------
% Day 3's maxPoints=200 filter (below, in sihLidarClusterDetections.m)
% assumed the ground plane always segments into ONE big ~9822-point blob.
% The Day 4 diagnostic (sihLidarDiag.m) showed the dominant false-track
% source instead sits at road height (median cluster centroid z = 0.00m,
% vs 0.83m for real matched actors) even though several of these clusters
% are as large as 176 points -- comparable to, or bigger than, real actor
% clusters (median 27, up to 196 points). A point-count ceiling alone
% can't reliably tell these apart, so height is used as the actual
% discriminator: a cluster is rejected as ground if it's both LOW (mean
% height below GroundZMax) AND FLAT (height std-dev below GroundZStdMax)
% -- the flatness check exists specifically so a real, low-profile object
% (e.g. glancing off a PushCart's lower body) isn't rejected on height
% alone if its points still span meaningful vertical extent.
% REASONED DEFAULTS, NOT YET VERIFIED against real per-point height
% distributions -- only the diagnostic's median/range summary stats were
% available when these were chosen. Re-check against the post-change
% "Missed objects" and "Per-actor tracking" numbers (not just false-track
% count) to confirm no real actor is being wrongly suppressed.
cfg.Lidar.GroundZMax    = 0.15;  % m, reject if mean cluster height below this
cfg.Lidar.GroundZStdMax = 0.10;  % m, AND cluster height std-dev below this

% ---------- Tracker tuning ----------
cfg.Tracker.AssignmentThreshold  = 60;    % Day 4, 3rd pass: was 40.
                                           % ConfirmationThreshold was ruled
                                           % out (see above). Diagnostic
                                           % then showed 95% of false
                                           % tracks (987/1036) sit a MEDIAN
                                           % of just 0.46m from an already-
                                           % matched real track -- i.e.
                                           % duplicate/split tracks of the
                                           % SAME real object, not ghosts
                                           % (only 5% were truly isolated).
                                           % Likely tied to Day 2's own
                                           % documented finding: vision/
                                           % radar report a point near an
                                           % object's near SURFACE (not
                                           % centroid), offset scaling with
                                           % size -- two sensors on the
                                           % same real object can legitimately
                                           % disagree by close to this much,
                                           % which a too-tight gate reads as
                                           % two different objects. 60 is a
                                           % MODERATE loosening, not
                                           % extreme -- the real risk is
                                           % merging genuinely distinct
                                           % close objects (TwoWheeler/
                                           % Pedestrian crossing cases,
                                           % already this tracker's known
                                           % weak point without JPDA -- see
                                           % sihCreateTracker.m). Report
                                           % only for this pass -- if
                                           % trackYield drops meaningfully
                                           % below ~100% or TwoWheeler/
                                           % Pedestrian per-actor tracking
                                           % falls, that's overcorrection
                                           % (real distinct objects merging),
                                           % back off toward 50 rather than
                                           % push further same direction.
cfg.Tracker.ConfirmationThreshold = [3 5];% Day 4: RULED OUT as a lever.
                                           % [4 6] (~same real-frame window
                                           % as [3 5], see below) gave
                                           % BIT-IDENTICAL results. [9 15]
                                           % (~5 real frames, same 0.6
                                           % ratio, isolating window-size)
                                           % barely moved anything and what
                                           % little it moved was SLIGHTLY
                                           % WORSE (false tracks/frame
                                           % 4.1774->4.2016, missed objects
                                           % unchanged to 4dp). Root cause
                                           % identified: the Day 1 ghost-ID
                                           % fix calls tracker() up to 3x
                                           % per real 0.1s frame (once per
                                           % sensor), so [M N] counts
                                           % sensor-updates not frames --
                                           % but even at ~5 real frames'
                                           % worth, no benefit appeared.
                                           % Conclusion: the false tracks
                                           % surviving the LiDAR ground
                                           % filter are genuinely
                                           % persistent -- not surviving on
                                           % a technicality of confirmation
                                           % window length. Reverted to
                                           % original value; next lever to
                                           % investigate is AssignmentThreshold
                                           % (below), since persistence this
                                           % strong points at how tracks get
                                           % CREATED, not how long they're
                                           % allowed to live.
cfg.Tracker.DeletionThreshold     = [5 5];% M-of-N to delete
cfg.Tracker.ClutterDensity        = 1e-6; % JPDA only
cfg.Tracker.InitVelStd            = 6;    % m/s, initial velocity uncertainty

% ---------- Post-hoc confirmed-track deduplication (Day 4) ----------
% 95% of false tracks were found to sit a median 0.46m from an already-
% matched real track (candidate duplicates, not ghosts -- see
% runPerceptionStub.m's NearMatched diagnostic). AssignmentThreshold
% loosening (40->60) barely helped, so this promotes that SAME
% distance+class check from diagnostic to an actual post-hoc filter on
% the tracker's confirmed list, rather than relying on the tracker's own
% assignment gate to prevent duplicates from forming in the first place.
% 0.6m chosen deliberately tighter than a round 1.0m: comfortably above
% the measured 0.46m median (catches the real pattern) but well short of
% real-world separation between genuinely distinct nearby actors -- the
% failure mode to avoid is merging two real crossing agents (TwoWheeler/
% Pedestrian, this tracker's known weak point without JPDA).
%
% RESULT at 0.6m: false tracks/frame 4.18->2.67 (-36%), yield 116%->94%,
% TwoWheeler/Pedestrian/Pedestrian2 per-actor tracking each lost only 1-2
% of 248 frames (checked -- not the merging failure mode above).
%
% RESIDUAL, NOT YET ACTED ON: of what's still false after this filter,
% 93% is STILL NearMatched, but median distance moved from 0.46m to
% 0.80m -- the close-in duplicates are gone, what's left sits farther
% out. A second dedup pass at a larger cap could plausibly catch more,
% but with less margin against the same crossing-agent-merge risk -- if
% this gets revisited, redo the SAME TwoWheeler/Pedestrian per-actor
% check at whatever new distance is tried, don't assume it's still safe.
cfg.Tracker.DedupCapDistance      = 0.6;  % m, see runPerceptionStub.m

% ---------- Contract-format extras ----------
cfg.MinSpeedForHeading = 0.25;  % m/s below this, heading is unreliable
cfg.StationaryHeadingVar = (pi/3)^2; % rad^2 injected when nearly stationary

% ---------- Planning region of interest (validation metric only) ----------
% Restricts the validation summary to objects that actually matter for
% planning, so long-range/behind-ego misses (a coverage-geometry artefact,
% not a tracker defect) don't dominate the aggregate numbers.
cfg.ROI.MaxRange   = 40;    % m
cfg.ROI.MaxAbsAzim = 90;    % deg, forward half-plane only

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
