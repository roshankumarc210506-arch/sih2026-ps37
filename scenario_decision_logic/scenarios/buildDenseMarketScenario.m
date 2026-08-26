function scenario = buildDenseMarketScenario()
%BUILDDENSEMARKETSCENARIO Scenario 4/5 -- dense mixed-traffic market
%   area.
%
%   Deliberately the most agent-dense scene -- the stress test for M1's
%   tracker under closely-spaced crossing agents and for M3's
%   inflation-radius costmap in tight space.
%
%   Returns a drivingScenario object.

    scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 30);

    roadCenters = [0 0 0; 80 0 0];
    road(scenario, roadCenters, 'Lanes', lanespec(1, 'Width', 4.5), ...
        'Name', 'MarketStreet');   % narrow market street

    egoCar = addEgoVehicle(scenario, [0 0 0], 0);
    egoWaypoints = [0 0 0; 40 0 0; 80 0 0];
    egoSpeed = 10 * ones(3, 1) / 3.6;   % ~10 km/h crawl speed
    trajectory(egoCar, egoWaypoints, egoSpeed);

    % --- Dense placeholder agent set ------------------------------------
    % ClassID here is the ADT actor class, not M1's AgentClass -- see
    % buildVillageRoadScenario for the full note.

    pushcartPositions = [15 1.8 0; 25 -1.6 0; 45 1.7 0; 60 -1.5 0];
    for i = 1:size(pushcartPositions, 1)
        actor(scenario, 'ClassID', 3, 'Length', 1.6, 'Width', 0.9, ...
            'Height', 1.2, 'Position', pushcartPositions(i, :), ...
            'Name', sprintf('Pushcart%d', i));
    end

    % NOTE: vehicle() only accepts ClassID 1 (Car) or 2 (Truck) on this
    % MATLAB release -- use actor() for the two-wheeler placeholders
    % instead, same as the pushcart/pedestrian actors above/below.
    twoWheelerPositions = [20 -2.0 0; 50 2.0 0];
    for i = 1:size(twoWheelerPositions, 1)
        tw = actor(scenario, 'ClassID', 3, 'Length', 1.9, 'Width', 0.7, ...
            'Height', 1.5, 'Position', twoWheelerPositions(i, :), 'Yaw', 0, ...
            'Name', sprintf('TwoWheeler%d', i));
        trajectory(tw, [twoWheelerPositions(i, :); ...
            twoWheelerPositions(i, :) + [30 0 0]], 12 * ones(2, 1) / 3.6);
    end

    pedestrianPositions = [10 1.0 0; 30 -1.2 0; 55 1.5 0; 70 -1.0 0];
    for i = 1:size(pedestrianPositions, 1)
        ped = actor(scenario, 'ClassID', 4, 'Length', 0.5, 'Width', 0.5, ...
            'Height', 1.7, 'Position', pedestrianPositions(i, :), ...
            'Name', sprintf('Pedestrian%d', i));
        crossTo = pedestrianPositions(i, :) + ...
            [0 -2 * sign(pedestrianPositions(i, 2)) 0];
        trajectory(ped, [pedestrianPositions(i, :); crossTo], 1.2 * ones(2, 1));
    end
end
