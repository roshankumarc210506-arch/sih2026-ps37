function [ids, firstSeenTimes, numIds] = computeM1FirstSeenTimes(m1MatFilePath, maxUniqueIds)
%COMPUTEM1FIRSTSEENTIMES  For every track id that ever appears in M1's
%   real perception log, find the sim time it FIRST became valid.
%   Needed because M2's prediction timeseries use a per-agent RELATIVE
%   clock (each starts at 0.00 regardless of when the agent actually
%   entered) - confirmed empirically, not assumed.
%
%   CHANGED (Simulink-safety pass): was containers.Map, which is not
%   usable inside a MATLAB Function block. Returns fixed-size parallel
%   arrays (ids, firstSeenTimes) + numIds instead, matching the same
%   fixed-array-plus-count convention already used everywhere else in
%   this pipeline (NumWaypoints, NumTracks, etc). Use lookupFirstSeenTime.m
%   in place of the old isKey/offsetMap(id) Map indexing.
%
%   NOTE on sizing: this counts UNIQUE ids across the ENTIRE session
%   (248 timesteps), NOT how many are valid in any single frame - so it
%   is NOT bounded by cfg.MaxTracks (40). A busy scenario with many
%   agents entering/exiting over time can accumulate far more unique ids
%   than are ever simultaneously valid. maxUniqueIds is a separate,
%   deliberately generous capacity - if exceeded, this errors loudly
%   rather than silently dropping ids (a silently dropped id here would
%   corrupt M2 prediction alignment for that agent, not just this
%   function's own output).
%
%   maxUniqueIds : (optional, default 300) fixed capacity for the
%                  returned arrays. NOT verified against the real
%                  m1_perception_day1.mat's true unique-id count yet -
%                  recommended to check that directly
%                  (numel(unique(cell2mat(arrayfun(@(v) [v.tracks(1:v.num_tracks).id], ...
%                  S.perceptionData.signals.values, 'UniformOutput', false)')))
%                  or similar) and right-size this rather than trust 300
%                  as more than a safe placeholder.
%
%   ids, firstSeenTimes : maxUniqueIds x 1 fixed-size arrays. Only
%                         entries 1:numIds are real; the rest is
%                         zero-padding.
%   numIds              : how many leading entries are real.

if nargin < 2 || isempty(maxUniqueIds)
    maxUniqueIds = 300;
end

S1 = load(m1MatFilePath);
ids            = zeros(maxUniqueIds, 1);
firstSeenTimes = zeros(maxUniqueIds, 1);
numIds         = 0;

vals = S1.perceptionData.signals.values;
for t = 1:numel(vals)
    rec = vals(t);
    liveIds = [rec.tracks(1:rec.num_tracks).id];
    for idx = 1:numel(liveIds)
        id = liveIds(idx);

        alreadySeen = false;
        for j = 1:numIds
            if ids(j) == id
                alreadySeen = true;
                break;
            end
        end

        if ~alreadySeen
            numIds = numIds + 1;
            if numIds > maxUniqueIds
                error('computeM1FirstSeenTimes:TooManyIds', ...
                    ['Found more than %d unique track ids across the session - ' ...
                     'increase maxUniqueIds. This is a loud failure, not a silent ' ...
                     'truncation: dropping an id here would silently corrupt M2 ' ...
                     'prediction alignment for that agent.'], maxUniqueIds);
            end
            ids(numIds)            = id;
            firstSeenTimes(numIds) = rec.timestamp;
        end
    end
end
end