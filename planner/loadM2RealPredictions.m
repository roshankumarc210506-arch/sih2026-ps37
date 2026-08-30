function [predictions, numPredictions, predTime] = loadM2RealPredictions(matFilePath, globalTimeQuery, ids, firstSeenTimes, numIds, maxTracks)
%LOADM2REALPREDICTIONS  Read one GLOBAL-time snapshot of M2's real
%   predictions. Requires ids/firstSeenTimes/numIds (from
%   computeM1FirstSeenTimes.m) to convert each agent's own relative
%   clock into global sim time - M2's timeseries all start at 0.00
%   regardless of when the agent actually entered the scene (confirmed
%   empirically).
%
%   CHANGED (Simulink-safety pass):
%   - predictions is now a FIXED-SIZE maxTracks x 1 struct array
%     (default 40, matching perception/sihConfig.m's CURRENT cfg.MaxTracks
%     - confirmed via repo inspection, NOT the stale 20 M2's own
%       Prediction/sihConfig.m still uses; see note below), not a
%     growable one - was struct('id',{},...) + predictions(end+1)=...,
%     which cannot run inside a MATLAB Function block.
%   - offsetMap (containers.Map) replaced by the ids/firstSeenTimes/
%     numIds fixed-array triple from computeM1FirstSeenTimes.m, looked
%     up via lookupFirstSeenTime.m instead of isKey/offsetMap(id).
%
%   FLAG, not fixed here (raise with M2/M6, same item M1 already flagged
%   in commit 6ef54b5): M2's OWN real prediction file
%   (Prediction/m2_predictions_day1.mat) is a 20x1 struct array - it
%   physically only has 20 slots today, regardless of maxTracks passed
%   here. Sizing this function's OUTPUT to 40 does not manufacture 20
%   more real predictions; it just means slots beyond the real file's 20
%   are honestly empty/unfilled (numPredictions will reflect only what's
%   actually found), consistent with the existing "no prediction found
%   -> 0 uncertainty growth" fallback already in buildRealOccupancyMap.m.
%   This does NOT fix the cross-team size mismatch - it just means M3's
%   own code no longer breaks in Simulink while that mismatch gets
%   resolved on M2's side.
%
%   matFilePath      : path to m2_predictions_day1.mat.
%   globalTimeQuery  : global sim time to sample at.
%   ids, firstSeenTimes, numIds : from computeM1FirstSeenTimes.m.
%   maxTracks        : (optional, default 40) fixed size for the
%                      returned predictions array. Pass cfg.MaxTracks
%                      explicitly once a single shared config value
%                      exists across M1/M2/M3 - see FLAG above.
%
%   predictions    : maxTracks x 1 fixed-size struct array
%                    {id, predicted_positions[10x2], uncertainty_radius[10x1]}.
%                    Only entries 1:numPredictions are real; the rest is
%                    zero-padding (id=0, which is never a valid real id
%                    per AgentClass/track id conventions - 0 is reserved
%                    for Unknown-class handling elsewhere, not an id).
%   numPredictions : how many leading entries are real.
%   predTime       : echoes globalTimeQuery, for bookkeeping.

if nargin < 6 || isempty(maxTracks)
    maxTracks = 40;
end

S = load(matFilePath);
pd = S.predictionData;
nSlots = numel(pd);   % the REAL file's own slot count (today: 20 - see FLAG above)

predictions = repmat(struct('id', 0, 'predicted_positions', zeros(10,2), ...
    'uncertainty_radius', zeros(10,1)), maxTracks, 1);
numPredictions = 0;
predTime = globalTimeQuery;

for k = 1:nSlots
    idVal = pd(k).id.Data(1);
    if idVal == 0
        continue;
    end

    [firstSeen, found] = lookupFirstSeenTime(ids, firstSeenTimes, numIds, idVal);
    if ~found
        warning('loadM2RealPredictions:NoOffset', ...
            'id %d has no known M1 first-seen time - skipping.', idVal);
        continue;
    end

    relTime = globalTimeQuery - firstSeen;   % global -> this agent's own clock
    tAxis = pd(k).id.Time;
    if relTime < tAxis(1) || relTime > tAxis(end)
        continue;   % this agent's prediction window doesn't cover the requested global time
    end

    [~, sIdx] = min(abs(tAxis - relTime));
    validVal = pd(k).valid.Data(sIdx);
    if ~validVal
        continue;
    end

    numPredictions = numPredictions + 1;
    if numPredictions > maxTracks
        error('loadM2RealPredictions:TooManyPredictions', ...
            ['Found more than maxTracks (%d) valid predictions at this timestep - ' ...
             'increase maxTracks. Loud failure, not silent truncation.'], maxTracks);
    end

    predictions(numPredictions) = struct( ...
        'id', idVal, ...
        'predicted_positions', pd(k).predicted_positions.Data(:,:,sIdx), ...
        'uncertainty_radius',  pd(k).uncertainty_radius.Data(:,:,sIdx));
end
end