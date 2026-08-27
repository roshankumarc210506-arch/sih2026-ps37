function predictions = predictMotionCVCTRBlock(tracks, num_tracks, N, dt) %#codegen
%PREDICTMOTIONCVCTRBLOCK Simulink-safe version of predictMotionCVCTR.
%
% Use this INSIDE a MATLAB Function block. Differences from the plain
% MATLAB version (predictMotionCVCTR.m), both required by Simulink/codegen:
%
%   1. Output is a FIXED-SIZE 1x20 struct array, index-aligned with
%      `tracks` (predictions(i) corresponds to tracks(i)). No dynamic
%      growth (predictions(end+1) = ...) - Simulink requires output size
%      to be known at compile time.
%   2. Invalid/unused slots (i > num_tracks, or tracks(i).valid == false)
%      get id = 0 and all-zero position/radius, rather than being omitted.
%      Downstream blocks should check num_tracks / a mirrored valid flag,
%      same convention as the input bus.
%
% INPUTS  - same conventions as predictMotionCVCTR.m:
%   tracks     : 1x20 fixed-size struct array (SihTrackBus-shaped)
%   num_tracks : how many of the 20 slots are real
%   N, dt      : horizon steps / step size
%
% OUTPUT:
%   predictions : 1x20 struct array, each element:
%                   .id, .predicted_positions [N x 2], .uncertainty_radius [N x 1]

MAX_TRACKS = 20;
uncertaintyParams = classUncertaintyParams();

predictions = repmat(struct( ...
    'id', uint32(0), ...
    'predicted_positions', zeros(N, 2), ...
    'uncertainty_radius', zeros(N, 1)), 1, MAX_TRACKS);

for i = 1:MAX_TRACKS
    if i > num_tracks || ~tracks(i).valid
        continue
    end

    trk = tracks(i);

    % yawRate is not yet on the bus contract - hardcoded to 0 (pure CV)
    % until M1 adds it. The CTR branch below activates automatically
    % once a real yawRate value flows in here.
    yawRate = 0;

      [alpha, growthExponent, r0fallback] = lookupClassParams(uncertaintyParams, trk.class);

    pos = zeros(N, 2);
    rad = zeros(N, 1);

    x0 = trk.x; y0 = trk.y; h0 = trk.heading; v = trk.velocity;
    r0 = baseUncertaintyRadius(trk.covariance, r0fallback);

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

    predictions(i).id = trk.id;
    predictions(i).predicted_positions = pos;
    predictions(i).uncertainty_radius = rad;
end

end

% ----- local helpers (same logic as predictMotionCVCTR.m) -----

function [alpha, growthExponent, r0] = lookupClassParams(uncertaintyParams, agentClass)
idx = find([uncertaintyParams.agentClass] == agentClass, 1);
if isempty(idx)
    idx = find([uncertaintyParams.agentClass] == AgentClass.Pedestrian, 1);
end
alpha = uncertaintyParams(idx).alpha;
growthExponent = uncertaintyParams(idx).growthExponent;
r0 = uncertaintyParams(idx).r0;
end

