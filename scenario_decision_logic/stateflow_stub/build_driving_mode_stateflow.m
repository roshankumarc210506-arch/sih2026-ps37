function build_driving_mode_stateflow()
%BUILD_DRIVING_MODE_STATEFLOW Day 1 scaffold for the decision-logic chart.
%
%   Programmatically builds a Simulink model containing a Stateflow
%   chart with the 4 required modes (CRUISE/CAUTIOUS/YIELD/STOP) and
%   placeholder guard conditions, per the shared interface contract:
%
%       Stateflow output -> driving_mode enum, broadcast as bus signal;
%       consumed by M4 ONLY. Does NOT feed the occupancy map/inflation
%       radii.
%
%   Transitions are wired per the doc's decision drivers -- agent
%   density, presence of a high-risk agent (pedestrian/animal) in a
%   risk zone, and planner infeasibility -- but the THRESHOLDS are
%   placeholders (Stateflow chart Parameters) with NO hysteresis tuning
%   yet. That tuning is explicitly a Phase 1 (Days 2-4) task once
%   density/risk-zone signals are real instead of stubbed; this script
%   only needs to exist and run today.
%
%   Requires DrivingMode.m (same folder) to be on the MATLAB path.
%
%   Run this once; re-running closes and rebuilds the model from
%   scratch so it stays idempotent while you iterate.

    modelName = 'DecisionLogic_Stub';

    % --- Idempotent rebuild -------------------------------------------
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    chartBlockPath = [modelName '/DrivingModeChart'];
    add_block('sflib/Chart', chartBlockPath);

    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.Chart', 'Path', chartBlockPath);
    chart.ActionLanguage = 'MATLAB';

    % --- Data dictionary -------------------------------------------------
    % Inputs (all placeholders -- real signals land in Phase 1/2)
    dAgentDensity = Stateflow.Data(chart);
    dAgentDensity.Name = 'agent_density';
    dAgentDensity.Scope = 'Input';
    dAgentDensity.DataType = 'double';

    dRiskZoneAgent = Stateflow.Data(chart);
    dRiskZoneAgent.Name = 'risk_zone_high_risk_agent';
    dRiskZoneAgent.Scope = 'Input';
    dRiskZoneAgent.DataType = 'boolean';
    % NOTE: computed upstream of this chart (not inside it) as: any
    % tracked agent inside a defined risk zone whose class is
    % AgentClass.Pedestrian or AgentClass.Animal (M1's perception-bus
    % enum, in AgentClass.m -- separate from this chart's own
    % DrivingMode enum). Reference AgentClass.Pedestrian /
    % AgentClass.Animal by name wherever that upstream logic lives, per
    % M1's contract note -- never a string literal.

    dPlannerInfeasible = Stateflow.Data(chart);
    dPlannerInfeasible.Name = 'planner_infeasible';
    dPlannerInfeasible.Scope = 'Input';
    dPlannerInfeasible.DataType = 'boolean';

    % Output -- the actual contract signal
    dDrivingMode = Stateflow.Data(chart);
    dDrivingMode.Name = 'driving_mode';
    dDrivingMode.Scope = 'Output';
    dDrivingMode.DataType = 'DrivingMode';

    % Thresholds are hardcoded literals directly in the guard conditions
    % below (3 and 1.5) rather than Stateflow "Parameter" data objects --
    % this MATLAB release doesn't allow scripting an initial value onto
    % Parameter-scope chart data ("Parameters and in-place data do not
    % support initial values"). These are PLACEHOLDERS either way; when
    % you actually tune them in Phase 1, promote them to real tunable
    % parameters by hand in the chart (Model Explorer / Symbols pane --
    % add Data, set Scope = Parameter, give it a value there) instead of
    % leaving magic numbers in the guard text.

    % --- States ------------------------------------------------------
    sCruise = Stateflow.State(chart);
    sCruise.Position = [40 40 130 70];
    sCruise.LabelString = sprintf(['CRUISE\n' ...
        'entry: driving_mode = DrivingMode.CRUISE;']);

    sCautious = Stateflow.State(chart);
    sCautious.Position = [260 40 130 70];
    sCautious.LabelString = sprintf(['CAUTIOUS\n' ...
        'entry: driving_mode = DrivingMode.CAUTIOUS;']);

    sYield = Stateflow.State(chart);
    sYield.Position = [260 200 130 70];
    sYield.LabelString = sprintf(['YIELD\n' ...
        'entry: driving_mode = DrivingMode.YIELD;']);

    sStop = Stateflow.State(chart);
    sStop.Position = [40 200 130 70];
    sStop.LabelString = sprintf(['STOP\n' ...
        'entry: driving_mode = DrivingMode.STOP;']);

    % --- Default transition (chart starts in CRUISE) ------------------
    defaultTransition = Stateflow.Transition(chart);
    defaultTransition.Destination = sCruise;
    defaultTransition.DestinationOClock = 9;

    % --- Density-driven CRUISE <-> CAUTIOUS, with hysteresis gap ------
    tToCautious = Stateflow.Transition(chart);
    tToCautious.Source = sCruise;
    tToCautious.Destination = sCautious;
    tToCautious.LabelString = '[agent_density > 3]';   % PLACEHOLDER threshold
    tToCautious.SourceOClock = 3;
    tToCautious.DestinationOClock = 9;

    tToCruise = Stateflow.Transition(chart);
    tToCruise.Source = sCautious;
    tToCruise.Destination = sCruise;
    tToCruise.LabelString = '[agent_density < 1.5]';   % PLACEHOLDER threshold
    tToCruise.SourceOClock = 9;
    tToCruise.DestinationOClock = 3;

    % --- Risk-zone-driven -> YIELD (from CRUISE or CAUTIOUS) ----------
    tCruiseToYield = Stateflow.Transition(chart);
    tCruiseToYield.Source = sCruise;
    tCruiseToYield.Destination = sYield;
    tCruiseToYield.LabelString = '[risk_zone_high_risk_agent]';
    tCruiseToYield.SourceOClock = 6;
    tCruiseToYield.DestinationOClock = 12;

    tCautiousToYield = Stateflow.Transition(chart);
    tCautiousToYield.Source = sCautious;
    tCautiousToYield.Destination = sYield;
    tCautiousToYield.LabelString = '[risk_zone_high_risk_agent]';
    tCautiousToYield.SourceOClock = 6;
    tCautiousToYield.DestinationOClock = 12;

    tYieldToCautious = Stateflow.Transition(chart);
    tYieldToCautious.Source = sYield;
    tYieldToCautious.Destination = sCautious;
    tYieldToCautious.LabelString = '[~risk_zone_high_risk_agent]';
    tYieldToCautious.SourceOClock = 12;
    tYieldToCautious.DestinationOClock = 6;

    % --- Planner-infeasibility -> STOP, from every other state --------
    % (Stateflow has no implicit "any state" transition; wiring one
    % explicitly per source state is the standard idiom.)
    stopSources = [sCruise, sCautious, sYield];
    for i = 1:numel(stopSources)
        t = Stateflow.Transition(chart);
        t.Source = stopSources(i);
        t.Destination = sStop;
        t.LabelString = '[planner_infeasible]';
    end

    % STOP -> CRUISE recovery is intentionally left as a placeholder:
    % whether this should be automatic once planner_infeasible clears,
    % or require an explicit re-arm/handshake with M3's replanner, is a
    % real design decision -- flag it for the team, don't just guess.
    tStopRecover = Stateflow.Transition(chart);
    tStopRecover.Source = sStop;
    tStopRecover.Destination = sCruise;
    tStopRecover.LabelString = ...
        '[~planner_infeasible] % PLACEHOLDER -- confirm recovery semantics with M3/M4';
    tStopRecover.SourceOClock = 3;
    tStopRecover.DestinationOClock = 9;

    % --- Wire top-level ports so this is drop-in for M6's model -------
    add_block('simulink/Sources/In1', [modelName '/agent_density'], ...
        'Position', [40 40 70 60]);
    add_block('simulink/Sources/In1', [modelName '/risk_zone_high_risk_agent'], ...
        'Position', [40 100 70 120]);
    add_block('simulink/Sources/In1', [modelName '/planner_infeasible'], ...
        'Position', [40 160 70 180]);
    add_block('simulink/Sinks/Out1', [modelName '/driving_mode'], ...
        'Position', [400 100 430 120]);

    chartPos = get_param(chartBlockPath, 'Position');
    chartWidth = chartPos(3) - chartPos(1);
    chartHeight = chartPos(4) - chartPos(2);
    set_param(chartBlockPath, 'Position', [150 40 150 + chartWidth, 40 + chartHeight]);

    add_line(modelName, 'agent_density/1', 'DrivingModeChart/1', 'autorouting', 'on');
    add_line(modelName, 'risk_zone_high_risk_agent/1', 'DrivingModeChart/2', 'autorouting', 'on');
    add_line(modelName, 'planner_infeasible/1', 'DrivingModeChart/3', 'autorouting', 'on');
    add_line(modelName, 'DrivingModeChart/1', 'driving_mode/1', 'autorouting', 'on');

    fprintf(['Built %s with a 4-state driving_mode Stateflow chart ' ...
        '(CRUISE/CAUTIOUS/YIELD/STOP).\n'], modelName);
    fprintf(['All guard thresholds are PLACEHOLDERS -- retune once ' ...
        'agent_density / risk_zone_high_risk_agent / planner_infeasible ' ...
        'carry real signals (Phase 1, Days 2-4).\n']);
    fprintf('Save explicitly if you want to keep it: save_system(''%s'')\n', modelName);
end
