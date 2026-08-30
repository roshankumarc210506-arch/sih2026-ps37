function defineScenarioEgoPoseBus()
%DEFINESCENARIOEGOPOSEBUS Define the Simulink.Bus object required by the
%   Scenario Reader block's "Ego Vehicle Pose" input port when Source of
%   ego vehicle = Input port.
%
%   Required shape, per Simulink's own error message and MathWorks'
%   documented ActorPose struct (Create Driving Scenario Programmatically):
%     ActorID          uint32  1
%     Position         double  [3x1]  [x y z], world frame
%     Velocity         double  [3x1]  [vx vy vz]
%     Roll             double  1      deg
%     Pitch            double  1      deg
%     Yaw              double  1      deg  (NOTE: degrees, not radians --
%                                            SihEgoBus.yaw is radians, the
%                                            adapter egoBusToScenarioPose.m
%                                            converts)
%     AngularVelocity  double  [3x1]
%
%   Run this before opening/compiling M5_ScenarioReader.slx, e.g. add to
%   the model's PreLoadFcn callback, same pattern as sihDefineBuses.m.
%
%   Owner: M5. Not part of the team-wide interface contract (this bus is
%   internal plumbing for the Scenario Reader adapter, not a
%   cross-module signal) -- no notification needed per contract rules,
%   but documented here for anyone who opens this model fresh.

e(1) = Simulink.BusElement;
e(1).Name = 'ActorID';
e(1).DataType = 'uint32';
e(1).Dimensions = 1;

e(2) = Simulink.BusElement;
e(2).Name = 'Position';
e(2).DataType = 'double';
e(2).Dimensions = 3;

e(3) = Simulink.BusElement;
e(3).Name = 'Velocity';
e(3).DataType = 'double';
e(3).Dimensions = 3;

e(4) = Simulink.BusElement;
e(4).Name = 'Roll';
e(4).DataType = 'double';
e(4).Dimensions = 1;

e(5) = Simulink.BusElement;
e(5).Name = 'Pitch';
e(5).DataType = 'double';
e(5).Dimensions = 1;

e(6) = Simulink.BusElement;
e(6).Name = 'Yaw';
e(6).DataType = 'double';
e(6).Dimensions = 1;

e(7) = Simulink.BusElement;
e(7).Name = 'AngularVelocity';
e(7).DataType = 'double';
e(7).Dimensions = 3;

ScenarioEgoPoseBus = Simulink.Bus;
ScenarioEgoPoseBus.Elements = e;
ScenarioEgoPoseBus.Description = ['Ego pose shape required by Scenario ' ...
    'Reader''s Input port mode for ego vehicle. Matches MATLAB''s ' ...
    'native drivingScenario ActorPose struct. Fed by ' ...
    'egoBusToScenarioPose.m, which converts SihEgoBus (M1''s shape, ' ...
    'x/y/yaw[rad]/velocity/Timestamp) into this shape.'];

assignin('base', 'ScenarioEgoPoseBus', ScenarioEgoPoseBus);

fprintf('[M5] ScenarioEgoPoseBus created (matches Scenario Reader''s required Input-port shape).\n');
end