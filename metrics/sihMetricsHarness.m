function results = sihMetricsHarness()
%SIHMETRICSHARNESS  Initializes the empty results table schema used
%across all 5 scenarios x 5+ runs (Phase 3 target). Call this once, then
%sihLogRun() after every closed-loop run once real scenarios exist.
%
% Owner: M6.

    results = sihInitResults();
    assignin('base', 'sih_results', results);
    disp('Empty results table created in base workspace as ''sih_results''.');
end

function T = sihInitResults()
    T = table('Size', [0 8], ...
        'VariableTypes', {'string','double','double','double','double','double','logical','string'}, ...
        'VariableNames', {'Scenario','RunID','ReplanLatency_s','PathCurvature_max', ...
                           'PathJerk_rms','CompletionTime_s','Completed','FailureMode'});
end

function results = sihLogRun(results, scenario, runID, replanLatency, curvatureMax, jerkRms, completionTime, completed, failureMode)
%SIHLOGRUN  Append one run's metrics to the results table.
% failureMode: '' if completed==true, else e.g. 'collision','deadlock','oob'
    if nargin < 9
        failureMode = "";
    end
    newRow = table(string(scenario), runID, replanLatency, curvatureMax, ...
        jerkRms, completionTime, completed, string(failureMode), ...
        'VariableNames', results.Properties.VariableNames);
    results = [results; newRow];
end