function offsetMap = computeM1FirstSeenTimes(m1MatFilePath)
%COMPUTEM1FIRSTSEENTIMES  For every track id that ever appears in M1's
%   real perception log, find the sim time it FIRST became valid.
%   Needed because M2's prediction timeseries use a per-agent RELATIVE
%   clock (each starts at 0.00 regardless of when the agent actually
%   entered) - confirmed empirically, not assumed.
S1 = load(m1MatFilePath);
offsetMap = containers.Map('KeyType','double','ValueType','double');
vals = S1.perceptionData.signals.values;
for t = 1:numel(vals)
    rec = vals(t);
    liveIds = [rec.tracks(1:rec.num_tracks).id];
    for id = liveIds
        if ~isKey(offsetMap, id)
            offsetMap(id) = rec.timestamp;
        end
    end
end
end