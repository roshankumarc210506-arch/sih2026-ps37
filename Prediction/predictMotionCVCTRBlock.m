function predictions = predictMotionCVCTRBlock(tracks, num_tracks, dt) %#codegen
%PREDICTMOTIONCVCTRBLOCK CV/CTR predictor, output shaped to match M6's
% locked SihPredictionBus contract: predictions.agents(1:40), .num_agents,
% .timestamp. Horizon N=20 to match buses/sihDefineBuses.m N_HORIZON.

MAX_TRACKS = 40;
N = 20;
uncertaintyParams = classUncertaintyParams();

persistent prevHeading prevId hasHistory
if isempty(prevHeading)
    prevHeading = zeros(1, MAX_TRACKS);
    prevId = zeros(1, MAX_TRACKS, 'uint32');
    hasHistory = false(1, MAX_TRACKS);
end

predictions.agents = repmat(struct( ...
    'id', uint32(0), ...
    'predicted_positions', zeros(N, 2), ...
    'uncertainty_radius', zeros(N, 1), ...
    'valid', false), MAX_TRACKS, 1);
predictions.num_agents = uint32(num_tracks);
predictions.timestamp = 0;

for i = 1:MAX_TRACKS
    if i > num_tracks || ~tracks(i).valid
        continue
    end

    trk = tracks(i);

    yawRate = 0;
    if hasHistory(i) && prevId(i) == trk.id
        headingDiff = mod(double(trk.heading) - prevHeading(i) + pi, 2*pi) - pi;
        if abs(headingDiff) > 1e-3
            yawRate = headingDiff / dt;
        end
    end
    prevHeading(i) = double(trk.heading);
    prevId(i) = trk.id;
    hasHistory(i) = true;

    [alpha, growthExponent, r0fallback] = lookupClassParams(uncertaintyParams, trk.class);

    pos = zeros(N, 2);
    rad = zeros(N, 1);

    x0 = double(trk.x); y0 = double(trk.y); h0 = double(trk.heading); v = double(trk.velocity);
    r0 = baseUncertaintyRadius(trk.covariance, r0fallback);
    if abs(h0) < 1e-9 && abs(v) < 1e-9
        r0 = r0 * 2;
    end

    for k = 1:N
        t = k * dt;
        if abs(yawRate) > 1e-6
            theta_k = h0 + yawRate * t;
            x_k = x0 + (v / yawRate) * (sin(theta_k) - sin(h0));
            y_k = y0 - (v / yawRate) * (cos(theta_k) - cos(h0));
        else
            x_k = x0 + v * cos(h0) * t;
            y_k = y0 + v * sin(h0) * t;
        end
        pos(k, :) = [x_k, y_k];

        if growthExponent == 1
            rad(k) = r0 + alpha * t;
        else
            rad(k) = r0 + alpha * (t ^ growthExponent);
        end
    end

    predictions.agents(i).id = trk.id;
    predictions.agents(i).predicted_positions = pos;
    predictions.agents(i).uncertainty_radius = rad;
    predictions.agents(i).valid = true;
end

end

% ----- local helpers -----

function [alpha, growthExponent, r0] = lookupClassParams(uncertaintyParams, agentClass)
alpha = 0; growthExponent = 1; r0 = 0;
found = false;
for k = 1:numel(uncertaintyParams)
    if uncertaintyParams(k).agentClass == agentClass
        alpha = uncertaintyParams(k).alpha;
        growthExponent = uncertaintyParams(k).growthExponent;
        r0 = uncertaintyParams(k).r0;
        found = true;
        break
    end
end
if ~found
    for k = 1:numel(uncertaintyParams)
        if uncertaintyParams(k).agentClass == AgentClass.Pedestrian
            alpha = uncertaintyParams(k).alpha;
            growthExponent = uncertaintyParams(k).growthExponent;
            r0 = uncertaintyParams(k).r0;
            break
        end
    end
end
end

function r0 = baseUncertaintyRadius(covariance, fallback)
P = double(covariance(1:2, 1:2));
if any(isnan(P(:))) || any(isinf(P(:)))
    r0 = fallback;
    return
end
eigVals = real(eig((P + P.') / 2));
maxEig = max(eigVals);
if maxEig <= 0
    r0 = fallback;
else
    r0 = sqrt(maxEig);
end
r0 = real(r0);
end