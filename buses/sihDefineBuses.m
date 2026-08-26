function sihDefineBuses()
%SIHDEFINEBUSES  Single entry point for ALL Simulink.Bus objects on the
%SIH 2026 PS37 project. Run in model InitFcn (Model Properties >
%Callbacks > InitFcn: sihDefineBuses) so every subsystem gets the same
%definitions with zero copy-paste drift.
%
% Ownership: M1 owns AgentClass.m + sihCreateBuses.m (SihTrackBus,
% SihPerceptionBus, SihEgoBus). M6 owns this file, which CALLS M1's
% script first and then adds the buses M1 does not own (prediction,
% plan, driving mode, control). There is exactly ONE script that
% actually defines each bus name -- this file must never redefine a
% bus M1's script already created. If you need a field changed on a
% bus M1 owns, that's a change to sihCreateBuses.m, not this file.

    assert(exist('AgentClass', 'class') == 8, ...
        'AgentClass enum not found on path. Add /buses to your MATLAB path.');
    assert(exist('sihCreateBuses', 'file') > 0, ...
        'sihCreateBuses.m (M1''s bus script) not found on path.');

    % ---- Single source of truth for SihTrackBus / SihPerceptionBus / SihEgoBus ----
    sihCreateBuses();

    %% ---- Prediction bus (M2 owner) ----
    % Fixed horizon N steps, fixed max agent count to match SihTrackBus (20)
    N_HORIZON = 20;      % prediction steps, tune later
    MAX_TRACKS = 20;     % must match M1's fixed track array length

    predAgent(1) = Simulink.BusElement;
    predAgent(1).Name = 'id';
    predAgent(1).DataType = 'uint32';
    predAgent(1).Dimensions = 1;

    predAgent(2) = Simulink.BusElement;
    predAgent(2).Name = 'predicted_positions';
    predAgent(2).DataType = 'double';
    predAgent(2).Dimensions = [N_HORIZON 2]; % [x y] per step

    predAgent(3) = Simulink.BusElement;
    predAgent(3).Name = 'uncertainty_radius';
    predAgent(3).DataType = 'double';
    predAgent(3).Dimensions = N_HORIZON;

    predAgent(4) = Simulink.BusElement;
    predAgent(4).Name = 'valid';
    predAgent(4).DataType = 'boolean';
    predAgent(4).Dimensions = 1;

    SihPredictedAgentBus = Simulink.Bus;
    SihPredictedAgentBus.Elements = predAgent;
    assignin('base', 'SihPredictedAgentBus', SihPredictedAgentBus);
    clear predAgent

    predTop(1) = Simulink.BusElement;
    predTop(1).Name = 'agents';
    predTop(1).DataType = 'Bus: SihPredictedAgentBus';
    predTop(1).Dimensions = MAX_TRACKS; % fixed-length array, mirrors SihTrackBus

    predTop(2) = Simulink.BusElement;
    predTop(2).Name = 'num_agents';
    predTop(2).DataType = 'uint32';
    predTop(2).Dimensions = 1;

    predTop(3) = Simulink.BusElement;
    predTop(3).Name = 'timestamp';
    predTop(3).DataType = 'double';
    predTop(3).Dimensions = 1;

    SihPredictionBus = Simulink.Bus;
    SihPredictionBus.Elements = predTop;
    assignin('base', 'SihPredictionBus', SihPredictionBus);
    clear predTop

    %% ---- Plan / EgoState / ControlCmd buses (M4 owner) ----
    % SihPlanBus, SihEgoStateBus, SihControlCmdBus are all defined in
    % control/createM4BusObjects.m -- that script is the single source of
    % truth for these three, matching M4's tested Model Reference block
    % (nonvirtual bus ports require an EXACT type match, so there must be
    % only one place these get defined). Do not redefine them here.
    %
    % OPEN ITEM as of Day 1: MAX_WAYPOINTS is set to 5000 in M4's script,
    % asserted as "per M3's contract" but not yet confirmed by M3 directly
    % -- M3's real output already hit 223 waypoints on a placeholder map.
    % Directions dtype/shape (int8 row vs double column) is also still an
    % open question for M3 to settle. Do not treat 5000/double as final
    % until M3 confirms.
    assert(exist('createM4BusObjects', 'file') > 0, ...
        'createM4BusObjects.m (M4''s bus script) not found on path.');
    createM4BusObjects();

    %% ---- Driving mode bus (M5 owner, M4-only consumer) ----
    % Define the enum once here so M5 and M4 share the same type.
    if ~(exist('DrivingMode', 'class') == 8)
        warning(['DrivingMode enum not found. Ask M5 to commit DrivingMode.m ' ...
                 '(Simulink.IntEnumType subclass: CRUISE/CAUTIOUS/YIELD/STOP) today.']);
    end

    mode(1) = Simulink.BusElement;
    mode(1).Name = 'driving_mode';
    mode(1).DataType = 'Enum: DrivingMode';
    mode(1).Dimensions = 1;

    mode(2) = Simulink.BusElement;
    mode(2).Name = 'timestamp';
    mode(2).DataType = 'double';
    mode(2).Dimensions = 1;

    SihDrivingModeBus = Simulink.Bus;
    SihDrivingModeBus.Elements = mode;
    assignin('base', 'SihDrivingModeBus', SihDrivingModeBus);
    clear mode

    disp('sihDefineBuses: called sihCreateBuses() [M1] and createM4BusObjects() [M4], then created SihPredictionBus and SihDrivingModeBus [M6].');
    disp('NOTE: SihPlanBus.Waypoints max rows (5000) and Directions dtype are unconfirmed by M3 as of Day 1 -- do not treat as final.');
    disp('All bus objects now defined in base workspace via a single call: sihDefineBuses().');
end