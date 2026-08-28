function [predictions, numPredictions, predTime] = loadM2RealPredictions(matFilePath, globalTimeQuery, offsetMap)
%LOADM2REALPREDICTIONS  Read one GLOBAL-time snapshot of M2's real
%   predictions. Requires offsetMap (from computeM1FirstSeenTimes) to
%   convert each agent's own relative clock into global sim time -
%   M2's timeseries all start at 0.00 regardless of when the agent
%   actually entered the scene (confirmed empirically).

S = load(matFilePath);
pd = S.predictionData;
nSlots = numel(pd);

predictions = struct('id', {}, 'predicted_positions', {}, 'uncertainty_radius', {});
predTime = globalTimeQuery;

for k = 1:nSlots
    idVal = pd(k).id.Data(1);
    if idVal == 0
        continue;
    end
    if ~isKey(offsetMap, idVal)
        warning('loadM2RealPredictions:NoOffset', ...
            'id %d has no known M1 first-seen time - skipping.', idVal);
        continue;
    end

    relTime = globalTimeQuery - offsetMap(idVal);   % global -> this agent's own clock
    tAxis = pd(k).id.Time;
    if relTime < tAxis(1) || relTime > tAxis(end)
        continue;   % this agent's prediction window doesn't cover the requested global time
    end
    [~, sIdx] = min(abs(tAxis - relTime));

    validVal = pd(k).valid.Data(sIdx);
    if ~validVal
        continue;
    end

    predictions(end+1) = struct( ...
        'id', idVal, ...
        'predicted_positions', pd(k).predicted_positions.Data(:,:,sIdx), ...
        'uncertainty_radius',  pd(k).uncertainty_radius.Data(:,:,sIdx)); %#ok<AGROW>
end
numPredictions = numel(predictions);
end