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
    roadWidth = 5.0;
    laneSpec = lanespec(1, 'Width', roadWidth);
    road(scenario, roadCenters, 'Lanes', laneSpec, 'Name', 'VillageRoad');
    % TODO (M5, manual in DSD GUI): drag the road edges irregular, add
    % shoulder clutter/encroachment props if the app's library has them.

    egoCar = addEgoVehicle(scenario, [0 -1.5 0], 0);
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
