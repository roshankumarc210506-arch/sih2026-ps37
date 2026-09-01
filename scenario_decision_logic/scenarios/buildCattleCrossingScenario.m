function scenario = buildCattleCrossingScenario()
%BUILDCATTLECROSSINGSCENARIO Scenario 5/5 -- sudden cattle-crossing
%   event.
%
%   Straight road, ego at speed, an animal that starts stationary
%   off-road and then crosses abruptly partway through the run (the
%   "sudden" part -- timed to appear once ego is already close).
%
%   Returns a drivingScenario object.

    scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 20);

    roadCenters = [0 0 0; 150 0 0];
    road(scenario, roadCenters, 'Lanes', lanespec(2, 'Width', 3.5), ...
        'Name', 'OpenRoad');

    egoCar = addEgoVehicle(scenario, [0 -1.75 0], 0);
    assert(egoCar.ActorID == 1, 'egoBusToScenarioPose.m hardcodes ActorID=1 for ego -- this scenario violated the addEgoVehicle-first convention.');
    egoWaypoints = [0 -1.75 0; 150 -1.75 0];
    egoSpeed = 45 * ones(2, 1) / 3.6;   % ~45 km/h open-road speed
    trajectory(egoCar, egoWaypoints, egoSpeed);

    % Animal actor: waits off-road, then darts across right as ego
    % closes in. At ~45 km/h (12.5 m/s) ego reaches ~x=75 m around
    % t=6s -- time the crossing to start just before that so it's
    % genuinely "sudden" rather than a slow, easily-predicted drift.
    animal = actor(scenario, 'ClassID', 4, 'Length', 1.8, 'Width', 0.6, ...
        'Height', 1.3, 'Position', [75 8 0], 'Name', 'Cattle1');

   animalWaypoints = [75 8 0; 75 7 0; 77 2 0; 73 -1 0; 75 -7 0; 75 -8 0];
   animalTimes = [0 5.0 5.8 6.6 7.5 8.0];   % holds off-road, then darts across
   % with a startled zigzag (forward dart, then jerk back) right as it
   % crosses the lane -- was a straight perpendicular line before, which
   % looked too smooth/predictable for "sudden erratic animal" behavior.
    trajectory(animal, animalWaypoints, animalTimes);
    % NOTE: uses the (waypoints, times) trajectory() form so the
    % "hold, then dart" timing is explicit rather than inferred from a
    % speed vector -- double-check this signature against your ADT
    % release; some versions expect (waypoints, speed) and derive time
    % from arc length instead.
end
