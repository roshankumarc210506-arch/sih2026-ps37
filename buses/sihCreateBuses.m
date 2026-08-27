function [trackBus, perceptionBus, egoBus] = sihCreateBuses(cfg)
%SIHCREATEBUSES  Build the Simulink Bus Objects for the perception contract.
%
%   sihCreateBuses(sihConfig());
%
%   Creates in the base workspace:
%     SihTrackBus       - ONE agent: {id, class, x, y, heading, velocity,
%                         covariance, valid}
%     SihPerceptionBus  - what M2/M3/M6 actually subscribe to:
%                         .tracks     [MaxTracks x 1] of SihTrackBus
%                         .num_tracks uint32
%                         .timestamp  double (s)
%     SihEgoBus         - ego pose in WORLD frame, published separately so
%                         the perception contract itself is untouched.
%
%   WHY the extra fields (tell the team):
%     * Simulink buses are FIXED SIZE. A variable-length struct array of
%       tracks is not expressible. Hence a fixed [MaxTracks x 1] array +
%       num_tracks + a per-element `valid` flag. Consumers must loop
%       1:num_tracks, or skip elements where valid == false.
%     * `class` is an enum, not a string (see AgentClass.m).
%     * `timestamp` lets M3 compute data age at the costmap boundary.

if nargin < 1 || isempty(cfg) || ~isfield(cfg,'MaxTracks')
    cfg.MaxTracks = 20;   % matches sihConfig.m's current default.
    % Deliberately NOT calling sihConfig() here — this file lives in
    % buses/ and must not depend on path resolution picking the
    % right sihConfig.m when more than one exists on the MATLAB
    % path (confirmed today: perception/ and Prediction/ both have
    % one). MaxTracks is the only field this function needs; keep
    % it local rather than reaching out.
end

% ---------------- Per-track bus ----------------
e = Simulink.BusElement.empty;

e(1)            = Simulink.BusElement;
e(1).Name       = 'id';
e(1).DataType   = 'uint32';
e(1).Dimensions = 1;
e(1).Description= 'Persistent track ID from the tracker';

e(2)            = Simulink.BusElement;
e(2).Name       = 'class';
e(2).DataType   = 'Enum: AgentClass';
e(2).Dimensions = 1;
e(2).Description= 'Fused agent class';

e(3)            = Simulink.BusElement;
e(3).Name       = 'x';
e(3).DataType   = 'double';
e(3).Dimensions = 1;
e(3).Unit       = 'm';
e(3).Description= 'Longitudinal position, ego frame (forward +)';

e(4)            = Simulink.BusElement;
e(4).Name       = 'y';
e(4).DataType   = 'double';
e(4).Dimensions = 1;
e(4).Unit       = 'm';
e(4).Description= 'Lateral position, ego frame (LEFT +)';

e(5)            = Simulink.BusElement;
e(5).Name       = 'heading';
e(5).DataType   = 'double';
e(5).Dimensions = 1;
e(5).Unit       = 'rad';
e(5).Description= 'Course angle, CCW from ego +x, wrapped to [-pi,pi]';

e(6)            = Simulink.BusElement;
e(6).Name       = 'velocity';
e(6).DataType   = 'double';
e(6).Dimensions = 1;
e(6).Unit       = 'm/s';
e(6).Description= 'Speed magnitude (non-negative)';

e(7)            = Simulink.BusElement;
e(7).Name       = 'covariance';
e(7).DataType   = 'double';
e(7).Dimensions = [4 4];
e(7).Description= '4x4 covariance over [x, y, heading, velocity]';

e(8)            = Simulink.BusElement;
e(8).Name       = 'valid';
e(8).DataType   = 'boolean';
e(8).Dimensions = 1;
e(8).Description= 'True if this slot holds a live track';

trackBus = Simulink.Bus;
trackBus.Elements    = e;
trackBus.Description = 'SIH PS37 single tracked agent (M1 output element)';
assignin('base', 'SihTrackBus', trackBus);

% ---------------- Top-level perception bus ----------------
p = Simulink.BusElement.empty;

p(1)            = Simulink.BusElement;
p(1).Name       = 'tracks';
p(1).DataType   = 'Bus: SihTrackBus';
p(1).Dimensions = cfg.MaxTracks;

p(2)            = Simulink.BusElement;
p(2).Name       = 'num_tracks';
p(2).DataType   = 'uint32';
p(2).Dimensions = 1;
p(2).Description= 'Number of valid entries at the FRONT of tracks';

p(3)            = Simulink.BusElement;
p(3).Name       = 'timestamp';
p(3).DataType   = 'double';
p(3).Dimensions = 1;
p(3).Unit       = 's';
p(3).Description= 'Simulation time this track list is valid for';

perceptionBus = Simulink.Bus;
perceptionBus.Elements    = p;
perceptionBus.Description = 'SIH PS37 perception output (M1 -> M2/M3/M6)';
assignin('base', 'SihPerceptionBus', perceptionBus);

% ---------------- Ego pose bus (side channel) ----------------
g = Simulink.BusElement.empty;
names = {'x','y','yaw','velocity','Timestamp'};
units = {'m','m','rad','m/s','s'};
for k = 1:5
    g(k)            = Simulink.BusElement;
    g(k).Name       = names{k};
    g(k).DataType   = 'double';
    g(k).Dimensions = 1;
    g(k).Unit       = units{k};
end
g(5).Description = 'Simulation time, not wall-clock -- same concern already raised to M3 about navClock.';
egoBus = Simulink.Bus;
egoBus.Elements    = g;
egoBus.Description = 'Ego pose in WORLD frame (for ego<->world transforms)';
assignin('base', 'SihEgoBus', egoBus);

fprintf('[M1] Buses created: SihTrackBus, SihPerceptionBus, SihEgoBus (MaxTracks=%d)\n', cfg.MaxTracks);
end
