function [tracks, numTracks, predictions, egoPose] = generateFakeTracksAndPredictions()
%GENERATEFAKETRACKSANDPREDICTIONS  Stand-in for real M1/M2 .mat data.
%   Matches the locked contract shape exactly, so swapping in real data
%   later should require zero changes to buildRealOccupancyMap.m.
%   NOTE: 'class' here is a plain string, not the real AgentClass enum,
%   since M1's AgentClass.m hasn't been pulled into this folder yet.
%   classToRiskField uses char(agentClass), which works for both.

egoPose = [10, 20, 0];   % world frame: at (10,20), facing +x (east)

raw(1) = struct('id',1,'class','Pedestrian','x',15,'y', 2,'heading',pi/2,'velocity',1.0);
raw(2) = struct('id',2,'class','Animal',    'x',25,'y',-3,'heading',0,   'velocity',0.5);
raw(3) = struct('id',3,'class','Car',       'x',30,'y', 5,'heading',0,   'velocity',5.0);

numTracks = numel(raw);  
% fixed-length 20-slot array per M1's bus contract
tracks = repmat(struct('id',0,'class','','x',0,'y',0,'heading',0, ...
    'velocity',0,'covariance',zeros(4),'valid',false), 20, 1);
for k = 1:numTracks
    tracks(k).id         = raw(k).id;
    tracks(k).class      = raw(k).class;
    tracks(k).x          = raw(k).x;
    tracks(k).y          = raw(k).y;
    tracks(k).heading    = raw(k).heading;
    tracks(k).velocity   = raw(k).velocity;
    tracks(k).covariance = eye(4) * 0.1;
    tracks(k).valid      = true;
end

% predictions, matched by id -- per-class growth rates from M2's answer
N = 10; dt = 0.2;   % 2.0 s horizon, per M2's current stub default
tHoriz = (1:N)' * dt;
params.Pedestrian    = struct('alpha',0.70,'exponent',0.7);
params.Animal        = struct('alpha',0.90,'exponent',0.6);
params.Car           = struct('alpha',0.15,'exponent',1.0);
params.TwoWheeler    = struct('alpha',0.35,'exponent',0.9);  
params.AutoRickshaw  = struct('alpha',0.30,'exponent',0.9);
params.PushCart      = struct('alpha',0.50,'exponent',0.8);   
r0 = 0.3;

predictions = struct('id',{},'predicted_positions',{},'uncertainty_radius',{});
for k = 1:numTracks
    t = raw(k);
    predXY = [t.x + t.velocity*cos(t.heading)*tHoriz, ...
              t.y + t.velocity*sin(t.heading)*tHoriz];
    p = params.(char(t.class));
    predictions(k).id                 = t.id;
    predictions(k).predicted_positions = predXY;
    predictions(k).uncertainty_radius  = r0 + p.alpha * tHoriz.^p.exponent;
end
end