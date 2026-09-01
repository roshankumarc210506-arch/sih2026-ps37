function sihBuildTopModel()
%SIHBUILDTOPMODEL  Builds sih_top_model.slx: 5 subsystems wired per the
%locked interface contract. Control is real (M4's Model Reference block);
%Perception, Prediction, GlobalPlanner, DecisionLogic are confirmed dummy
%stubs for Day 1, matching today's team decisions.
%
% PREREQUISITE: run sihDefineBuses() first in this MATLAB session.
%
% Owner: M6.
%
% RESOLVED (confirmed against M4_Control/ego_state port): SihEgoStateBus was
% DIFFERENT bus objects, never reconciled. Control gets its own
% dedicated ego_state stub below, NOT fed from Perception's real EgoOut.

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
    'sihSetupNlobj(); ' ...
        'perceptionStub = Simulink.Bus.createMATLABStruct(''SihPerceptionBus''); ' ...
        'egoStub = Simulink.Bus.createMATLABStruct(''SihEgoBus''); ' ...
        'planStub = Simulink.Bus.createMATLABStruct(''SihPlanBus''); ' ...
        'modeStub = DrivingMode.STOP; ' ]);
    set_param(modelName, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.1');

    perceptionStub = Simulink.Bus.createMATLABStruct('SihPerceptionBus');
    egoStub        = Simulink.Bus.createMATLABStruct('SihEgoBus');
    planStub       = Simulink.Bus.createMATLABStruct('SihPlanBus');
    modeStub       = DrivingMode.STOP;
    assignin('base', 'perceptionStub', perceptionStub);
    assignin('base', 'egoStub', egoStub);
    assignin('base', 'planStub', planStub);
    assignin('base', 'modeStub', modeStub);

    p = [modelName '/Perception'];
    add_block('simulink/Ports & Subsystems/Subsystem', p, 'Position', [30 30 160 130]);
    delete_block([p '/In1']);
    delete_block([p '/Out1']);
    add_block('simulink/Sources/Constant', [p '/PerceptionConst'], ...
        'Value', 'perceptionStub', 'OutDataTypeStr', 'Bus: SihPerceptionBus', 'Position', [30 20 190 60]);
    add_block('simulink/Sources/Constant', [p '/EgoConst'], ...
        'Value', 'egoStub', 'OutDataTypeStr', 'Bus: SihEgoBus', 'Position', [30 90 190 130]);
    add_block('simulink/Sinks/Out1', [p '/PerceptionOut'], 'Position', [260 20 290 60]);
    add_block('simulink/Sinks/Out1', [p '/EgoOut'], 'Position', [260 90 290 130]);
    add_line(p, 'PerceptionConst/1', 'PerceptionOut/1', 'autorouting', 'on');
    add_line(p, 'EgoConst/1', 'EgoOut/1', 'autorouting', 'on');

    pr = [modelName '/Prediction'];
    add_block('simulink/Ports & Subsystems/Subsystem', pr, 'Position', [340 30 470 90]);
    delete_block([pr '/In1']);
    delete_block([pr '/Out1']);
    add_block('simulink/Sources/In1', [pr '/PerceptionIn'], 'Position', [30 20 60 40]);
    add_block('simulink/Sources/Constant', [pr '/PredConst'], 'Value', '0', 'Position', [90 60 220 100]);
    add_block('simulink/Sinks/Out1', [pr '/PredictionOut'], 'Position', [260 60 290 100]);
    add_block('simulink/Sinks/Terminator', [pr '/PerceptionTerm'], 'Position', [90 20 110 40]);
    add_line(pr, 'PerceptionIn/1', 'PerceptionTerm/1', 'autorouting', 'on');
    add_line(pr, 'PredConst/1', 'PredictionOut/1', 'autorouting', 'on');

    gp = [modelName '/GlobalPlanner'];
    add_block('simulink/Ports & Subsystems/Subsystem', gp, 'Position', [340 130 470 220]);
    delete_block([gp '/In1']);
    delete_block([gp '/Out1']);
    add_block('simulink/Sources/In1', [gp '/PerceptionIn'], 'Position', [30 20 60 40]);
    add_block('simulink/Sources/In1', [gp '/PredictionIn'], 'Position', [30 60 60 80]);
    add_block('simulink/Sources/Constant', [gp '/PlanConst'], ...
        'Value', 'planStub', 'OutDataTypeStr', 'Bus: SihPlanBus', 'Position', [90 100 220 140]);
    add_block('simulink/Sinks/Out1', [gp '/PlanOut'], 'Position', [260 100 290 140]);
    add_block('simulink/Sinks/Terminator', [gp '/PerceptionTerm'], 'Position', [90 20 110 40]);
    add_block('simulink/Sinks/Terminator', [gp '/PredictionTerm'], 'Position', [90 60 110 80]);
    add_line(gp, 'PerceptionIn/1', 'PerceptionTerm/1', 'autorouting', 'on');
    add_line(gp, 'PredictionIn/1', 'PredictionTerm/1', 'autorouting', 'on');
    add_line(gp, 'PlanConst/1', 'PlanOut/1', 'autorouting', 'on');

    dl = [modelName '/DecisionLogic'];
    add_block('simulink/Ports & Subsystems/Subsystem', dl, 'Position', [340 250 470 310]);
    delete_block([dl '/In1']);
    delete_block([dl '/Out1']);
    add_block('simulink/Sources/In1', [dl '/PerceptionIn'], 'Position', [30 20 60 40]);
    add_block('simulink/Sources/Constant', [dl '/ModeConst'], ...
        'Value', 'modeStub', 'OutDataTypeStr', 'Enum: DrivingMode', 'Position', [90 60 220 100]);
    add_block('simulink/Sinks/Out1', [dl '/ModeOut'], 'Position', [260 60 290 100]);
    add_block('simulink/Sinks/Terminator', [dl '/PerceptionTerm'], 'Position', [90 20 110 40]);
    add_line(dl, 'PerceptionIn/1', 'PerceptionTerm/1', 'autorouting', 'on');
    add_line(dl, 'ModeConst/1', 'ModeOut/1', 'autorouting', 'on');

    ct = [modelName '/Control'];
    add_block('simulink/Ports & Subsystems/Model', ct, 'Position', [560 130 690 220]);
    set_param(ct, 'ModelNameDialog', 'M4_Control');


    add_line(modelName, 'Perception/1', 'Prediction/1', 'autorouting', 'on');
    add_line(modelName, 'Perception/1', 'GlobalPlanner/1', 'autorouting', 'on');
    add_line(modelName, 'Prediction/1', 'GlobalPlanner/2', 'autorouting', 'on');
    add_line(modelName, 'Perception/1', 'DecisionLogic/1', 'autorouting', 'on');
    add_line(modelName, 'GlobalPlanner/1', 'Control/1', 'autorouting', 'on');
    add_line(modelName, 'Perception/2', 'Control/2', 'autorouting', 'on');
    add_line(modelName, 'DecisionLogic/1', 'Control/3', 'autorouting', 'on');

    save_system(modelName, fullfile(pwd, 'model', [modelName '.slx']));
    fprintf('\nBuilt %s.slx.\n', modelName);
    fprintf('REAL today: Control (M4 Model Reference).\n');
    fprintf('STUB today (confirmed with owners): Perception, Prediction, GlobalPlanner, DecisionLogic.\n');
    fprintf('RESOLVED: Control/2 now fed from real SihEgoBus (Perception/2), not a stub.\n');
    fprintf('OPEN: speedCap_mps feedback loop not wired -- add when GlobalPlanner goes real.\n');
end


