function scenario = buildHighwayMergeScenario()
%BUILDHIGHWAYMERGESCENARIO Scenario 3/5 -- highway merge with
%   slow-moving vehicles.
%
%   Returns a drivingScenario object.

    scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 25);

    mainRoadCenters = [0 0 0; 100 0 0; 250 0 0];
    road(scenario, mainRoadCenters, 'Lanes', lanespec(2, 'Width', 3.5), ...
        'Name', 'MainCarriageway');

    rampCenters = [30 25 0; 60 12 0; 90 2 0];
    road(scenario, rampCenters, 'Lanes', lanespec(1, 'Width', 3.5), ...
        'Name', 'MergeRamp');

    egoCar = addEgoVehicle(scenario, [0 -1.75 0], 0);
    egoWaypoints = [0 -1.75 0; 100 -1.75 0; 250 -1.75 0];
    egoSpeed = 70 * ones(3, 1) / 3.6;   % ~70 km/h
    trajectory(egoCar, egoWaypoints, egoSpeed);

    % Slow-moving vehicle merging in from the ramp -- the scenario's
    % whole point, so it needs to actually be slow relative to ego.
    mergingTruck = vehicle(scenario, 'ClassID', 2, 'Length', 6.5, 'Width', 2.3, ...
        'Position', [30 25 0], 'Yaw', -50, 'Name', 'MergingTruck1');
    trajectory(mergingTruck, rampCenters, 25 * ones(3, 1) / 3.6);   % ~25 km/h

    % Faster car already ahead on the main road, to force a realistic
    % gap-acceptance decision during the merge.
    leadCar = vehicle(scenario, 'ClassID', 1, 'Length', 4.5, 'Width', 1.8, ...
        'Position', [40 -1.75 0], 'Yaw', 0, 'Name', 'LeadCar1');
    trajectory(leadCar, [40 -1.75 0; 250 -1.75 0], 60 * ones(2, 1) / 3.6);
end
