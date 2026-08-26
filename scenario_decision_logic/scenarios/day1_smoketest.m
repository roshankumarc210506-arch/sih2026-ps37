%% Day 1 smoke test -- build all 5 scenarios headlessly, no GUI popups.
% Confirms every scenario file runs end-to-end (roads + ego + actors
% construct without error, trajectories are well-formed) before you
% open anything visually. This is the automatable check M6 will want
% wired into the top-level integration harness in Phase 1+.

builders = {
    @buildVillageRoadScenario,      'Village road'
    @buildUrbanIntersectionScenario,'Urban intersection'
    @buildHighwayMergeScenario,     'Highway merge'
    @buildDenseMarketScenario,      'Dense market'
    @buildCattleCrossingScenario,   'Cattle crossing'
};

fprintf('--- Day 1 scenario smoke test (%s) ---\n', datestr(now));
allOk = true;
for i = 1:size(builders, 1)
    name = builders{i, 2};
    try
        scenario = builders{i, 1}();
        actorList = actorPoses(scenario);
        fprintf('[OK]   %-20s  %d actor(s) (incl. ego), StopTime=%.1fs\n', ...
            name, numel(actorList), scenario.StopTime);
    catch ME
        allOk = false;
        fprintf('[FAIL] %-20s  %s\n', name, ME.message);
    end
end

if allOk
    fprintf('\nAll 5 scenarios built cleanly. Open one visually with, e.g.:\n');
    fprintf('    drivingScenarioDesigner(buildVillageRoadScenario())\n');
else
    fprintf('\nAt least one scenario failed to build -- fix before Day 1 exit checkpoint.\n');
end
