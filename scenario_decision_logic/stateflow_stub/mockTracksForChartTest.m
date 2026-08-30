%MOCKTRACKSFORCHARTTEST Mock SihTrackBus-shaped data for testing
%   computeDecisionSignals.m wiring into DrivingModeChart.
%
%   Creates three test cases in the base workspace:
%     mockTracks_lowDensity   - 1 car, far away -> low density, no risk
%     mockTracks_highDensity  - 5 agents nearby -> triggers CAUTIOUS
%     mockTracks_pedestrianRisk - pedestrian close to ego -> triggers YIELD
%
%   Run this script, then feed one of these + its matching num_tracks
%   into the MATLAB Function block driving the chart (or call
%   computeDecisionSignals directly to sanity check before wiring).
%
%   Fields match SihTrackBus exactly (buses/sihCreateBuses.m):
%   id, class, x, y, heading, velocity, covariance, valid

MAX_TRACKS = 20;   % matches M1's real MaxTracks

% ---------- helper to build one blank track ----------
function t = blankTrack()
t.id = uint32(0);
t.class = AgentClass.Car;   % overwritten per-track below
t.x = 0; t.y = 0; t.heading = 0; t.velocity = 0;
t.covariance = zeros(4,4);
t.valid = false;
end

% ============================================================
% Case 1: Low density, no risk -- expect CRUISE to stay CRUISE
% ============================================================
tracks = repmat(blankTrack(), 1, MAX_TRACKS);
tracks(1).id = uint32(1);
tracks(1).class = AgentClass.Car;
tracks(1).x = 50; tracks(1).y = 0;   % far outside both zones (30m/15m radii)
tracks(1).velocity = 10;
tracks(1).valid = true;

mockTracks_lowDensity = tracks;
mockNumTracks_lowDensity = uint32(1);

% ============================================================
% Case 2: High density -- 5 valid agents within 30m -> CAUTIOUS
% ============================================================
tracks = repmat(blankTrack(), 1, MAX_TRACKS);
positions = [10 5; 15 -5; 20 8; 12 -10; 25 3];   % all within 30m of origin
for k = 1:5
    tracks(k).id = uint32(k);
    tracks(k).class = AgentClass.Car;
    tracks(k).x = positions(k,1);
    tracks(k).y = positions(k,2);
    tracks(k).velocity = 8;
    tracks(k).valid = true;
end

mockTracks_highDensity = tracks;
mockNumTracks_highDensity = uint32(5);

% ============================================================
% Case 3: Pedestrian within 15m risk zone -> YIELD
% ============================================================
tracks = repmat(blankTrack(), 1, MAX_TRACKS);
tracks(1).id = uint32(1);
tracks(1).class = AgentClass.Pedestrian;
tracks(1).x = 8; tracks(1).y = 2;    % within 15m risk radius
tracks(1).velocity = 1.2;
tracks(1).valid = true;

mockTracks_pedestrianRisk = tracks;
mockNumTracks_pedestrianRisk = uint32(1);

% ---------- quick sanity check when run directly ----------
fprintf('--- Case 1 (low density) ---\n');
[d1, r1] = computeDecisionSignals(mockTracks_lowDensity, mockNumTracks_lowDensity);
fprintf('agent_density=%d, risk_zone_high_risk_agent=%d (expect 0, 0)\n', d1, r1);

fprintf('--- Case 2 (high density) ---\n');
[d2, r2] = computeDecisionSignals(mockTracks_highDensity, mockNumTracks_highDensity);
fprintf('agent_density=%d, risk_zone_high_risk_agent=%d (expect 5, 0)\n', d2, r2);

fprintf('--- Case 3 (pedestrian risk) ---\n');
[d3, r3] = computeDecisionSignals(mockTracks_pedestrianRisk, mockNumTracks_pedestrianRisk);
fprintf('agent_density=%d, risk_zone_high_risk_agent=%d (expect 1, 1)\n', d3, r3);