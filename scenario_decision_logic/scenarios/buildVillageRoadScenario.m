function scenario = buildVillageRoadScenario()
%BUILDVILLAGEROADSCENARIO Scenario 1/5 -- unmarked Indian village road.
%   PS-required scene. Day 1 scaffold: geometry + a first-pass actor
%   set, good enough to smoke-test the perception->planner->control
%   chain today. Refine visual detail (informal encroachments, verge
%   clutter, unmarked edge blur) by hand in Driving Scenario Designer
%   once it's open -- that's a GUI job, not a scripting job.
%
%   Returns a drivingScenario object. Call openVillageRoadScenario to
%   also launch it in the Driving Scenario Designer app.
    scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 30);
% Narrow, winding road with a single loose lane -- no marked lanes,
% informal two-way use.
    roadCenters = [0 0 0;
                   25 4 0;
                   55 -2 0;
                   85 6 0;
                   120 0 0];
    roadWidth = 5.2;  % widened from 5.0 (Day 2, M3 footprint decision):
                       % M3's validated 3-circle collision check found the
                       % real minimum width was 5.140m at x=53.04 (near the
                       % sharp bend between centerline points (25,4) and
                       % (55,-2)), ~12cm short of the 1.5m lane-offset
                       % target. +20cm gives margin above the measured
                       % shortfall rather than a zero-margin exact fix.
    laneSpec = lanespec(1, 'Width', roadWidth);
    road(scenario, roadCenters, 'Lanes', laneSpec, 'Name', 'VillageRoad');
% TODO (M5, manual in DSD GUI): drag the road edges irregular, add
% shoulder clutter/encroachment props if the app's library has them.
    egoCar = addEgoVehicle(scenario, [0 -1.5 0], 0);
    assert(egoCar.ActorID == 1, 'egoBusToScenarioPose.m hardcodes ActorID=1 for ego -- this scenario violated the addEgoVehicle-first convention.');
    egoWaypoints = roadCenters + [0 -1.5 0];
    egoSpeed = 20 * ones(size(egoWaypoints, 1), 1) / 3.6;   % ~20 km/h
    trajectory(egoCar, egoWaypoints, egoSpeed);
% --- Placeholder agents ---------------------------------------------
% ClassID below is the ADT built-in actor class (1 Car, 3 Non-motor
% vehicle/bicycle-like, 4 Pedestrian) used just to get a plausible
% footprint/rendering in the scene. It is NOT M1's AgentClass enum
% from the perception bus (car/two-wheeler/auto-rickshaw/pushcart/
% pedestrian/animal) -- that enum only exists on the perception
% output, downstream of these scenario actors. Don't wire ClassID
% here into risk logic.
% Note (Day 2): Pushcart1/Pedestrian1/Animal1 positions below are all
% already outside the old 2.5m road half-width (verge/shoulder
% placements by design) -- the 5.0->5.2m widening does not require
% repositioning any of them.
    actor(scenario, 'ClassID', 3, 'Length', 1.6, 'Width', 0.9, ...
'Height', 1.2, 'Position', [40 2.5 0], 'Name', 'Pushcart1');
    pedestrian = actor(scenario, 'ClassID', 4, 'Length', 0.5, 'Width', 0.5, ...
'Height', 1.7, 'Position', [60 -6 0], 'Name', 'Pedestrian1');
    trajectory(pedestrian, [60 -6 0; 60 2 0], [1.2; 1.2]);   % ~1.2 m/s walk
% Stray animal grazing near the road edge -- stationary for Day 1;
% scenario 5 owns the "sudden crossing" behavior.
    actor(scenario, 'ClassID', 4, 'Length', 1.8, 'Width', 0.6, ...
'Height', 1.3, 'Position', [95 5 0], 'Name', 'Animal1');
end