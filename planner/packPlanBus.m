function planBus = packPlanBus(velPlan, planResult, cfg, resetSeq)
%PACKPLANBUS  Assemble the locked Plan-bus-to-MPC struct.
%
%   Schema (locked interface contract, field order preserved for a
%   Simulink Bus Object match):
%     {Waypoints[N x 4], NumWaypoints, Directions, SeqNum,
%      GenerationTimestamp, MapTimestamp, MapAgeAtPlan_s}
%
%   Waypoints/Directions are padded to cfg.Bus.MaxWaypoints rows. This is
%   what cfg.Bus.MaxWaypoints was defined for back in Step 2: Simulink
%   Bus Objects need a fixed signal size for codegen, so the array is
%   sized once at MaxWaypoints and NumWaypoints tells the consumer (M4)
%   how many leading rows are real data. Everything past row N is zero
%   padding, not a value - M4 must gate on NumWaypoints, not on array
%   length, or it will treat padding as an implicit stop-at-origin
%   waypoint.
%
%   resetSeq (optional, default false): pass true to reset the
%   persistent SeqNum counter to 1 at the start of a fresh run, so
%   repeated test runs in one MATLAB session don't inherit a stale
%   count from the previous one.

persistent seqCounter
if isempty(seqCounter)
    seqCounter = 0;
end
if nargin >= 4 && resetSeq
    seqCounter = 0;
end

N = velPlan.NumWaypoints;

if N > cfg.Bus.MaxWaypoints
    error('packPlanBus:TooManyWaypoints', ...
        'Plan has %d waypoints, exceeds Bus.MaxWaypoints (%d).', ...
        N, cfg.Bus.MaxWaypoints);
end

seqCounter = seqCounter + 1;

%% ---------- pad to fixed size ----------
Wp = zeros(cfg.Bus.MaxWaypoints, 4);
Wp(1:N, :) = velPlan.Waypoints;

Dr = zeros(cfg.Bus.MaxWaypoints, 1);
Dr(1:N) = velPlan.Directions;

%% ---------- assemble, field order matches the locked contract ----------
planBus = struct();
planBus.Waypoints           = Wp;                            % [MaxWaypoints x 4], rows 1:N valid
planBus.NumWaypoints        = uint32(N);
planBus.Directions          = Dr;                             % [MaxWaypoints x 1], rows 1:N valid
planBus.SeqNum              = uint32(seqCounter);
planBus.GenerationTimestamp = planResult.GenerationTimestamp;
planBus.MapTimestamp        = planResult.MapTimestamp;
planBus.MapAgeAtPlan_s      = planResult.MapAgeAtPlan_s;
end