function out = sihClassVoter(action, tracks, dets, cfg)
%SIHCLASSVOTER  Fuse per-detection class labels into one class per track.
%
%   sihClassVoter('reset');
%   classes = sihClassVoter('update', tracks, dets, cfg);
%
%   WHY this exists: the tracker fuses GEOMETRY, not semantics. A radar
%   detection says "Unknown", the camera says "TwoWheeler", the LiDAR says
%   "Pedestrian" — all on the same object. Taking whichever label arrived
%   last gives a class that flickers frame to frame, which makes M3's
%   risk-weighted costmap oscillate and M5's Stateflow chart chatter
%   between CRUISE and YIELD.
%
%   Approach: an exponentially-decayed vote histogram per track ID, with
%   each sensor weighted by its trustworthiness as a classifier
%   (cfg.Sensor(s).ClassAcc). Detections are attributed to the nearest
%   track within a gate. Simple, explainable, and defensible in the report.
%
%   Returns an AgentClass array, one entry per element of `tracks`.

persistent votes decay

if strcmp(action, 'reset')
    votes = containers.Map('KeyType','double','ValueType','any');
    decay = 0.92;
    out   = [];
    return
end

if isempty(votes)
    votes = containers.Map('KeyType','double','ValueType','any');
    decay = 0.92;
end

nT = numel(tracks);
out = repmat(AgentClass.Unknown, nT, 1);
if nT == 0, return; end

% ---- track positions in ego frame (state = [x vx y vy z vz]) ----
tp = zeros(nT, 2);
id = zeros(nT, 1);
for i = 1:nT
    s        = tracks(i).State;
    tp(i,:)  = [s(1), s(3)];
    id(i)    = double(tracks(i).TrackID);
end

% ---- decay existing votes so stale evidence fades ----
k = votes.keys;
for i = 1:numel(k)
    votes(k{i}) = votes(k{i}) * decay;
end

% ---- attribute each detection to its nearest track, then vote ----
gate = 4.0;  % m
for d = 1:numel(dets)
    z  = dets{d}.Measurement;
    dd = hypot(tp(:,1) - z(1), tp(:,2) - z(2));
    [dmin, iBest] = min(dd);
    if dmin > gate, continue; end

    cls = dets{d}.ObjectClassID;
    if cls == double(AgentClass.Unknown), continue; end   % no information

    w = cfg.Sensor(dets{d}.SensorIndex).ClassAcc;         % trust weighting

    key = id(iBest);
    if ~isKey(votes, key)
        votes(key) = zeros(1, cfg.NumClasses);
    end
    v = votes(key);
    v(cls + 1) = v(cls + 1) + w;    % +1 because Unknown == 0
    votes(key) = v;
end

% ---- argmax ----
for i = 1:nT
    if ~isKey(votes, id(i)), continue; end
    v = votes(id(i));
    [best, idx] = max(v);
    if best > 0.5      % require a minimum of accumulated evidence
        out(i) = AgentClass(idx - 1);
    end
end

% ---- forget vote histories for tracks that no longer exist ----
k = votes.keys;
for i = 1:numel(k)
    if ~any(id == k{i})
        remove(votes, k{i});
    end
end
end
