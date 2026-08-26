function egoCar = addEgoVehicle(scenario, position, yaw)
%ADDEGOVEHICLE Add the locked-spec ego vehicle to a drivingScenario.
%
%   Vehicle dimensions are LOCKED per the shared interface contract --
%   do not change without updating that contract doc + notifying all 6
%   members:
%       Wheelbase        2.5 m
%       Length           4.7 m
%       Width            1.8 m
%       Rear overhang    0.9 m   -> Front overhang = 4.7 - 2.5 - 0.9 = 1.3 m
%       Max steer        35 deg
%       MinTurningRadius 4.10 m  (= 1.15 x kinematic 3.57 m, 15% safety margin)
%
%   egoCar = addEgoVehicle(scenario, position, yaw)
%       scenario : a drivingScenario object
%       position : [x y z] scenario position, meters (default [0 0 0])
%       yaw      : initial heading, degrees (default 0)
%
%   NOTE: vehicle()'s built-in 'ClassID' (1=Car,2=Truck,3=Bicycle,
%   4=Pedestrian, ...) is the Automated Driving Toolbox actor class,
%   NOT the same thing as M1's custom AgentClass enum used on the
%   perception bus (car/two-wheeler/auto-rickshaw/pushcart/pedestrian/
%   animal). Don't conflate the two -- see the note in each scenario
%   script.

    wheelbase     = 2.5;
    bodyLength    = 4.7;
    bodyWidth     = 1.8;
    rearOverhang  = 0.9;
    frontOverhang = bodyLength - wheelbase - rearOverhang;  % 1.3 m

    if nargin < 2 || isempty(position)
        position = [0 0 0];
    end
    if nargin < 3 || isempty(yaw)
        yaw = 0;
    end

    egoCar = vehicle(scenario, ...
        'ClassID', 1, ...
        'Length', bodyLength, ...
        'Width', bodyWidth, ...
        'Height', 1.6, ...
        'RearOverhang', rearOverhang, ...
        'FrontOverhang', frontOverhang, ...
        'Position', position, ...
        'Yaw', yaw, ...
        'Name', 'Ego');

    % Informational sanity check against the locked min turning radius.
    % (plannerHybridAStar / NMPC own the real kinematic model -- this is
    % just a guard so nobody silently drifts off-contract while editing
    % scenario files.)
    minTurnRadius = 1.15 * (wheelbase / tand(35));
    assert(abs(minTurnRadius - 4.10) < 0.02, ...
        ['MinTurningRadius drifted from the locked contract value ' ...
         '(4.10 m). Got %.3f m -- check wheelbase/steer constants.'], ...
        minTurnRadius);
end
