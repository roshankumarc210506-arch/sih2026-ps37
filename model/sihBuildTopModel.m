function sihBuildTopModel()
%SIHBUILDTOPMODEL  Builds sih_top_model.slx: 5 subsystems wired per the
%locked interface contract. Control is real (M4's Model Reference block);
%Perception, Prediction, GlobalPlanner, DecisionLogic are confirmed dummy
%stubs for Day 1, matching today's team decisions.
%
% PREREQUISITE: run sihDefineBuses() first in this MATLAB session.
%
% Owner: M6. Re-run any time to rebuild from scratch after a bus or
% wiring change -- don't hand-patch the .slx and let it drift from this
% script.

    assert(evalin('base','exist(''SihPlanBus'',''var'')'), ...
        'Buses not found in base workspace. Run sihDefineBuses() first.');

    modelName = 'sih_top_model';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);
    set_param(modelName, 'InitFcn', [...
    'sihDefineBuses(); ' ...
    'perceptionStub = Simulink.Bus.createMATLABStruct(''SihPerceptionBus''); ' ...
    'egoStub = Simulink.Bus.createMATLABStruct(''SihEgoBus''); ' ]);
    set_param(modelName, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.1');
    % 0.1s matches M4's Control Ts directly, per M4's Day-1 request.

    %% ---- Perception: CONFIRMED dummy stub for Day 1 -----------------------
    % From Workspace on M1's real nested-struct data hit a real Simulink
    % limitation (structure-with-time format only accepts numeric/logical/
    % enum leaf values, not nested bus-shaped structs). Original Day-1 spec
    % explicitly allowed "dummy data OK" for Perception, so a zero-init
    % bus-typed Constant meets today's actual checkpoint.
    % TODO(M6, Phase 1 / Day 2): wire M1's real m1_perception_day1.mat
    % playback properly -- needs a MATLAB Function block or Dataset-based
    % approach, not From Workspace directly on nested struct data.
    perceptionStub = Simulink.Bus.createMATLABStruct('SihPerceptionBus');
    egoStub        = Simulink.Bus.createMATLABStruct('SihEgoBus');
    assignin('base', 'perceptionStub', perceptionStub);
    assignin('base', 'egoStub', egoStub);

    p = [modelName '/Perception'];
    add_block('simulink/Ports & Subsystems/Subsystem', p, 'Position', [30 30 160 130]);
    delete_block([p '/In1']);
    delete_block([p '/Out1']);
    add_block('simulink/Sources/Constant', [p '/PerceptionConst'], ...
        'Value', 'perceptionStub', 'Position', [30 20 190 60]);
    add_block('simulink/Sources/Constant', [p '/EgoConst'], ...
        'Value', 'egoStub', 'Position', [30 90 190 130]);
    add_block('simulink/Sinks/Out1', [p '/PerceptionOut'], 'Position', [260 20 290 60]);
    add_block('simulink/Sinks/Out1', [p '/EgoOut'], 'Position', [260 90 290 130]);
    add_line(p, 'PerceptionConst/1', 'PerceptionOut/1', 'autorouting', 'on');
    add_line(p, 'EgoConst/1', 'EgoOut/1', 'autorouting', 'on');

    %% ---- Prediction: CONFIRMED dummy stub (M2 has not reported) ----
    pr = [modelName '/Prediction'];
    add_block('simulink/Ports & Subsystems/Subsystem', pr, 'Position', [340 30 470 90]);
    delete_block([pr '/In1']);
    delete_block([pr '/Out1']);
    add_block('simulink/Sources/In1', [pr '/PerceptionIn'], 'Position', [30 20 60 40]);
    add_block('simulink/Sinks/Out1', [pr '/PredictionOut'], 'Position', [200 20 230 40]);
    % TODO(M2): replace with real predictor once M2 reports status.

    %% ---- GlobalPlanner: CONFIRMED dummy stub ----
    % M3's real pipeline is not Simulink-safe yet (variable-length structs,
    % string class field). M3 explicitly asked to keep this a stub through
    % Day 1 while converting internals to fixed-size/enum-typed.
    gp = [modelName '/GlobalPlanner'];
    add_block('simulink/Ports & Subsystems/Subsystem', gp, 'Position', [340 130 470 220]);
    delete_block([gp '/In1']);
    delete_block([gp '/Out1']);
    add_block('simulink/Sources/In1', [gp '/PerceptionIn'], 'Position', [30 20 60 40]);
    add_block('simulink/Sources/In1', [gp '/PredictionIn'], 'Position', [30 60 60 80]);
    add_block('simulink/Sinks/Out1', [gp '/PlanOut'], 'Position', [200 40 230 60]);
    % TODO(M3): swap for real pipeline once fixed-size/enum conversion done.

    %% ---- DecisionLogic: CONFIRMED dummy stub ----
    % M5's Stateflow file path not yet confirmed in repo.
    dl = [modelName '/DecisionLogic'];
    add_block('simulink/Ports & Subsystems/Subsystem', dl, 'Position', [340 250 470 310]);
    delete_block([dl '/In1']);
    delete_block([dl '/Out1']);
    add_block('simulink/Sources/In1', [dl '/PerceptionIn'], 'Position', [30 20 60 40]);
    add_block('simulink/Sinks/Out1', [dl '/ModeOut'], 'Position', [200 20 230 40]);
    % TODO(M5): swap for real Stateflow chart once file path confirmed + pushed.
    % IMPORTANT: real chart's getDefaultValue() must return DrivingMode.STOP
    % (fail-safe default) -- must not be changed to CRUISE for convenience.

    %% ---- Control: REAL, via M4's Model Reference block ----
    % Model Reference (not nested Subsystem) since M4_Control.slx is
    % binary and git can't merge it -- keeps M4's file and this top-level
    % model as separate files nobody collides on.
    ct = [modelName '/Control'];
    add_block('simulink/Ports & Subsystems/Model', ct, 'Position', [560 130 690 220]);
    set_param(ct, 'ModelNameDialog', 'M4_Control');
    % Port order per M4: 1=plan (SihPlanBus), 2=ego_state (SihEgoStateBus),
    % 3=driving_mode (Enum: DrivingMode). Output 1=cmd (SihControlCmdBus).

    %% ---- Wire top-level connections ----
    add_line(modelName, 'Perception/1', 'Prediction/1', 'autorouting', 'on');
    add_line(modelName, 'Perception/1', 'GlobalPlanner/1', 'autorouting', 'on');
    add_line(modelName, 'Prediction/1', 'GlobalPlanner/2', 'autorouting', 'on');
    add_line(modelName, 'Perception/1', 'DecisionLogic/1', 'autorouting', 'on');
    add_line(modelName, 'GlobalPlanner/1', 'Control/1', 'autorouting', 'on');
    add_line(modelName, 'Perception/2', 'Control/2', 'autorouting', 'on');
    add_line(modelName, 'DecisionLogic/1', 'Control/3', 'autorouting', 'on');

    % NOTE(M6, Day 1 open item -- flagged by M4): Control's third output
    % field, speedCap_mps, needs to route BACK to GlobalPlanner as
    % assignVelocityProfile's externalSpeedCap_mps argument -- a genuine
    % feedback loop, not wired on this first pass since GlobalPlanner is
    % still a stub. Add a Memory/unit-delay block on this signal when
    % GlobalPlanner goes real, so mode changes latch into the NEXT replan
    % cycle rather than creating a same-step algebraic loop.

    save_system(modelName, fullfile(pwd, 'model', [modelName '.slx']));
    fprintf('\nBuilt %s.slx.\n', modelName);
    fprintf('REAL today: Control (M4 Model Reference).\n');
    fprintf('STUB today (confirmed with owners): Perception, Prediction, GlobalPlanner, DecisionLogic.\n');
    fprintf('OPEN: speedCap_mps feedback loop not wired -- add when GlobalPlanner goes real.\n');
end