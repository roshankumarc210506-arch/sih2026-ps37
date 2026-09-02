function out = sihLidarDiag(action, kind, clusterSize, centroidZ, classNum)
%SIHLIDARDIAG  DIAGNOSTIC ONLY (Day 4). Tracks, across a full run, how
%   many LiDAR clusters end up in each of these buckets, plus (v2) the
%   point-count/height of every cluster, plus (v3) which real actor CLASS
%   a wrongfully-ground-rejected cluster belonged to -- so a class-level
%   regression (e.g. TwoWheeler, being low-profile) can be measured
%   directly instead of inferred from the before/after Per-actor table.
%   (v4) Also tracks total RAW cluster count per frame, BEFORE the
%   minPoints/maxPoints size filter -- added after the failed minDistance
%   0.75 attempt (severe regression, every bucket read 0) specifically to
%   see how pcsegdist's output actually behaves before touching that
%   threshold again, instead of guessing blind a second time.
%   (v5) Also tracks FilteredTooSmall/FilteredTooBig (clusters dropped by
%   the minPoints/maxPoints size gate, same Count/Sizes/Z/Classes shape as
%   every other bucket -- FilteredTooSmall's Classes field records the
%   nearby-actor class when one exists, same WrongfulReject-style check,
%   to see if minPoints=5 is wrongly dropping real small detections) and
%   MatchedPosError (real centroid-to-truth distance for every Matched
%   cluster, to inform the R noise-model question empirically instead of
%   guessing between the hardcoded 0.05m and cfg.Sensor(3).PosStd=0.14m).
%
%     Matched          attributed to a real actor within the 2.0m gate,
%                       and that actor wasn't already claimed by an
%                       earlier cluster this same frame.
%     OverSegmented     attributed to an actor ALREADY claimed by an
%                       earlier cluster this same frame.
%     Unattributed      no actor within the 2.0m gate at all, AND not
%                       ground-like by the height filter either.
%     GroundRejected    (v3) ground-like by the height filter, AND no
%                       real actor was within the 2.0m gate anyway --
%                       i.e. a "clean" ground rejection.
%     WrongfulReject     (v3) ground-like by the height filter, but a real
%                       actor WAS within the 2.0m gate -- this cluster
%                       would have been Matched/OverSegmented if the
%                       height filter didn't exist. classNum records
%                       which AgentClass that actor was (0=Unknown,
%                       1=Car, 2=TwoWheeler, 3=AutoRickshaw, 4=PushCart,
%                       5=Pedestrian, 6=Animal), so the tradeoff cost of
%                       the height filter is measured by class, not
%                       guessed at from Per-actor tracking deltas.
%
%   Purely observational -- does not change which detections
%   sihLidarClusterDetections.m emits or feeds to the tracker.
%
%   sihLidarDiag('reset');
%   sihLidarDiag('record', 'Matched', clusterSize, centroidZ);
%   sihLidarDiag('record', 'WrongfulReject', clusterSize, centroidZ, classNum);
%   sihLidarDiag('recordFrameCount', rawClusterCount);   % (v4) once per frame
%   out = sihLidarDiag('summary');
%     out.(kind).Count, .Sizes, .Z, .Classes (WrongfulReject only, meaningful)
%     out.RawClusterCounts (v4) -- one entry per frame, BEFORE size filtering

persistent data

kinds = {'Matched', 'OverSegmented', 'Unattributed', 'GroundRejected', 'WrongfulReject', 'FilteredTooSmall', 'FilteredTooBig'};

if isempty(data) || strcmp(action, 'reset')
    data = struct();
    for k = 1:numel(kinds)
        data.(kinds{k}) = struct('Count', 0, 'Sizes', [], 'Z', [], 'Classes', []);
    end
    data.RawClusterCounts = [];   % (v4)
    data.MatchedPosError  = [];   % (v5) real centroid-to-truth distance
                                   % for every Matched cluster -- direct
                                   % empirical LiDAR position accuracy,
                                   % to inform the R noise-model question
                                   % (cfg.Sensor(3).PosStd=0.14m vs the
                                   % hardcoded 0.05m currently used).
end

if nargin < 5
    classNum = NaN;
end

switch action
    case 'reset'
        out = [];
    case 'record'
        data.(kind).Count        = data.(kind).Count + 1;
        data.(kind).Sizes(end+1) = clusterSize; %#ok<AGROW>
        data.(kind).Z(end+1)     = centroidZ;   %#ok<AGROW>
        data.(kind).Classes(end+1) = classNum;  %#ok<AGROW>
        out = [];
    case 'recordFrameCount'
        % (v4) kind carries the raw cluster count here, not a bucket name
        % -- reusing the parameter slot rather than changing the function
        % signature, since this is diagnostic-only tooling.
        data.RawClusterCounts(end+1) = kind; %#ok<AGROW>
        out = [];
    case 'recordPosError'
        % (v5) kind carries the position-error value (metres) here, same
        % parameter-reuse pattern as recordFrameCount.
        data.MatchedPosError(end+1) = kind; %#ok<AGROW>
        out = [];
    case 'summary'
        out = data;
    otherwise
        error('sihLidarDiag:badAction', 'Unknown action "%s".', action);
end
end