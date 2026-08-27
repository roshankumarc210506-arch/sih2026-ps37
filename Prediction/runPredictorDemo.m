%% runPredictorDemo.m
% Stub end-to-end test of the classical CV/CTR predictor against dummy
% (or real, once wired up) perception data using M1's updated contract:
% fixed-length track array, AgentClass enum, ego-frame ISO 8855, 4x4
% covariance over [x, y, heading, velocity].

clear; clc; close all;

N  = 10;    % prediction horizon steps
dt = 0.2;   % s per step -> 2.0 s horizon (tune to taste)

% --- Input data ---
% Today: dummy data matching the fixed-length contract.
[tracks, num_tracks, ~] = generateDummyTracks();

% Once M1's .mat of real fused tracks is available, replace the two
% lines above with something like:
%   S = load('fused_tracks.mat');   % from M1
%   tracks = S.tracks; num_tracks = S.num_tracks;
% predictMotionCVCTR does not need to change either way.

uncertaintyParams = classUncertaintyParams();

predictions = predictMotionCVCTR(tracks, num_tracks, N, dt, uncertaintyParams);

%% quick visualization
colors = lines(num_tracks);

figure; hold on; axis equal; grid on;
xlabel('x (m, ego frame, forward)'); ylabel('y (m, ego frame, left)');
title('CV/CTR prediction stub - fixed-length bus contract (dotted = growing uncertainty)');

legendLabels = strings(numel(predictions), 1);

for i = 1:numel(predictions)
    trk = tracks(i); % predictions(i) corresponds to tracks(i) while all first num_tracks are valid
    pred = predictions(i);

    plot(trk.x, trk.y, 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
    plot(pred.predicted_positions(:,1), pred.predicted_positions(:,2), '-', ...
        'Color', colors(i,:), 'LineWidth', 1.5);

    for k = 1:N
        th = linspace(0, 2*pi, 30);
        cx = pred.predicted_positions(k,1) + pred.uncertainty_radius(k) * cos(th);
        cy = pred.predicted_positions(k,2) + pred.uncertainty_radius(k) * sin(th);
        plot(cx, cy, ':', 'Color', colors(i,:));
    end

    text(trk.x, trk.y, sprintf('  %s (id=%d)', char(trk.class), trk.id));
    legendLabels(i) = string(char(trk.class));
end

legend(legendLabels, 'Location', 'bestoutside');

%% sanity print - confirms pedestrians/animals grow faster than vehicles
fprintf('\nFinal-step uncertainty radius by class (t = %.1f s):\n', N*dt);
for i = 1:numel(predictions)
    fprintf('  Track %d (%-13s): %.2f m\n', ...
        predictions(i).id, char(tracks(i).class), predictions(i).uncertainty_radius(end));
end
