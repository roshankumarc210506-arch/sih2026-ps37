function scenario = buildUrbanIntersectionScenario()
%BUILDURBANINTERSECTIONSCENARIO Scenario 2/5 -- busy Indian urban
%   intersection, no traffic signals. PS-required scene.
%
%   "No signals" means: no trafficLight/intersection-control block is
%   added here -- right-of-way behavior comes from Stateflow's
%   YIELD/STOP logic + M4's control response, not from scripted signal
%   phases.
%
%   Returns a drivingScenario object.

    scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 25);

    roadCentersNS = [50 -50 0; 50 150 0];
    roadCentersEW = [-50 50 0; 150 50 0];
    road(scenario, roadCentersNS, 'Lanes', lanespec(2, 'Width', 3.25), ...
        'Name', 'NS_Road');
    road(scenario, roadCentersEW, 'Lanes', lanespec(2, 'Width', 3.25), ...
        'Name', 'EW_Road');
   % RESOLVED (M5, Day 2): left as an open, curb-free junction rather
   % than adding curb-return geometry -- road() doesn't support proper
   % rounded corners where two roads cross, and with no curbs at all,
   % there's nothing in the crossing for a turning vehicle to hit, so
   % the MinTurningRadius = 4.10 m constraint is trivially satisfied.
   % Matches the "no traffic signals, informal" theme anyway. Added a
   % stationary Car actor near the EW_Road east-arm shoulder to
   % represent illegal roadside parking/encroachment (see Actors in
   % urbanIntersection_M5.mat).

    egoCar = addEgoVehicle(scenario, [50 -50 0], 90);
    assert(egoCar.ActorID == 1, 'egoBusToScenarioPose.m hardcodes ActorID=1 for ego -- this scenario violated the addEgoVehicle-first convention.');
    egoWaypoints = [50 -50 0; 50 20 0; 50 40 0; 50 150 0];
    egoSpeed = 25 * ones(size(egoWaypoints, 1), 1) / 3.6;   % ~25 km/h approach
    trajectory(egoCar, egoWaypoints, egoSpeed);

    % --- Placeholder mixed traffic --------------------------------------
    % See buildVillageRoadScenario for the ClassID vs. AgentClass note.

    crossCar = vehicle(scenario, 'ClassID', 1, 'Length', 4.5, 'Width', 1.8, ...
        'Position', [-50 50 0], 'Yaw', 0, 'Name', 'CrossCar1');
    trajectory(crossCar, [-50 50 0; 20 50 0; 150 50 0], 30 * ones(3, 1) / 3.6);

    % NOTE: vehicle() only accepts ClassID 1 (Car) or 2 (Truck) on this
    % MATLAB release -- use actor() for the two-wheeler placeholder
    % instead, same as the pushcart/pedestrian actors below.
    twoWheeler = actor(scenario, 'ClassID', 3, 'Length', 1.9, 'Width', 0.7, ...
        'Height', 1.5, 'Position', [30 20 0], 'Yaw', 45, 'Name', 'TwoWheeler1');
    trajectory(twoWheeler, [30 20 0; 55 45 0; 80 70 0], 18 * ones(3, 1) / 3.6);

    vehicle(scenario, 'ClassID', 1, 'Length', 2.6, 'Width', 1.4, ...
        'Position', [58 58 0], 'Yaw', 200, 'Name', 'AutoRickshaw1');

    pedestrian = actor(scenario, 'ClassID', 4, 'Length', 0.5, 'Width', 0.5, ...
        'Height', 1.7, 'Position', [45 30 0], 'Name', 'Pedestrian1');
    trajectory(pedestrian, [45 30 0; 55 70 0], 1.3 * ones(2, 1));
end
