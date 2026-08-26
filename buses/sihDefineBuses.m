function sihDefineBuses()
%SIHDEFINEBUSES  Entry point for all SIH PS37 bus definitions.
%   Call once per model InitFcn. Perception buses are owned by M1
%   (sihCreateBuses); this adds buses other subsystems need.

cfg = sihConfig();
sihCreateBuses(cfg);   % SihTrackBus, SihPerceptionBus, SihEgoBus (M1's, do not touch)

% Plan bus needs MaxWaypoints from M3's planner config, not M1's sihConfig.
% Separate cfg variable on purpose -- these are two different config
% namespaces, don't merge them.
pcfg = planner_config();
N = pcfg.Bus.MaxWaypoints;   % 5000, read live from M3's config -- not hardcoded

%% ---------------- SihPlanBus ----------------
% Matches planner/packPlanBus.m field-for-field, verified against real
% source on 2026-08-26. Field order preserved to match the locked
% interface contract.
elems(1) = Simulink.BusElement;
elems(1).Name = 'Waypoints';
elems(1).Dimensions = [N 4];
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'NumWaypoints';
elems(2).Dimensions = 1;
elems(2).DataType = 'uint32';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Directions';
elems(3).Dimensions = [N 1];
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'SeqNum';
elems(4).Dimensions = 1;
elems(4).DataType = 'uint32';

elems(5) = Simulink.BusElement;
elems(5).Name = 'GenerationTimestamp';
elems(5).Dimensions = 1;
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'MapTimestamp';
elems(6).Dimensions = 1;
elems(6).DataType = 'double';

elems(7) = Simulink.BusElement;
elems(7).Name = 'MapAgeAtPlan_s';
elems(7).Dimensions = 1;
elems(7).DataType = 'double';

SihPlanBus = Simulink.Bus;
SihPlanBus.Elements = elems;
assignin('base', 'SihPlanBus', SihPlanBus);
clear elems

% NOTE: DrivingMode does NOT need a Simulink.Bus -- it's already a real
% Simulink.IntEnumType class (scenario_decision_logic/stateflow_stub/
% DrivingMode.m). Set the port type directly to "Enum: DrivingMode".
% Just make sure that file is on this model's path.

% TODO -- BLOCKED, do not fabricate:
%   SihPredictionBus  -- no M2 files exist in the repo yet. Field names
%     in the original interface contract are UNVERIFIED against real code.
%     Ping M2 for real output before defining this bus for real.
%   SihControlCmdBus  -- no M4 files exist in the repo yet either. Confirm
%     M4 has pushed, then read the real file before defining this.
end
