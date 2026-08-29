function [tracks, num_tracks, timestamp] = generateDummyTracks()
%GENERATEDUMMYTRACKS Dummy perception output matching M1's fixed-length
% bus contract, for standalone testing before swapping in M1's real
% exported .mat of fused-track data.
%
% Fixed-length array (mirrors SihTrackBus): 20 slots, only the first
% num_tracks are real, each with a .valid flag. Frame is ego-vehicle,
% ISO 8855 (x forward, y left, heading CCW). covariance is 4x4 over
% [x, y, heading, velocity].

MAX_TRACKS = 20;

tracks = repmat(makeEmptyTrack(), 1, MAX_TRACKS);

tracks(1) = makeTrack(1, AgentClass.Car,           10, 20, deg2rad(0),   8.0, 0.20);
tracks(2) = makeTrack(2, AgentClass.TwoWheeler,     5, 15, deg2rad(30),  6.0, 0.25);
tracks(3) = makeTrack(3, AgentClass.AutoRickshaw,  12, 18, deg2rad(-15), 4.5, 0.25);
tracks(4) = makeTrack(4, AgentClass.Pushcart,       8, 22, deg2rad(90),  1.2, 0.30);
tracks(5) = makeTrack(5, AgentClass.Pedestrian,    14, 16, deg2rad(180), 1.4, 0.20);
tracks(6) = makeTrack(6, AgentClass.Animal,         3, 25, deg2rad(45),  1.0, 0.30);

% Example: one track carries the optional yawRate field to demonstrate
% the predictor is already CTR-ready once the upstream bus supports it.
tracks(3).yawRate = deg2rad(10); % gently turning auto-rickshaw

num_tracks = 6;
timestamp = 0.0;

end

function t = makeTrack(id, agentClass, x, y, heading, velocity, posVar)
t = makeEmptyTrack();
t.id = id;
t.class = agentClass;
t.x = x;
t.y = y;
t.heading = heading;
t.velocity = velocity;
% 4x4 covariance over [x, y, heading, velocity] - diagonal placeholder,
% real data from M1 will have off-diagonal terms from the Jacobian.
t.covariance = diag([posVar, posVar, deg2rad(5)^2, 0.5^2]);
t.valid = true;
end

function t = makeEmptyTrack()
t.id = uint32(0);
t.class = AgentClass.Car;
t.x = 0;
t.y = 0;
t.heading = 0;
t.velocity = 0;
t.covariance = zeros(4,4);
t.valid = false;
end
