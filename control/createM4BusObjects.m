function createM4BusObjects()
%CREATEM4BUSOBJECTS Define the Simulink.Bus objects on M4's boundary.
%
%   Run once per MATLAB session before opening the Simulink model, or add
%   it to the model's PreLoadFcn callback so it runs automatically.
%
%   Creates in the base workspace:
%     SihPlanBus       - INPUT  from M3 (planner + velocity profiler)
%     SihEgoStateBus   - INTERNAL, vehicle dynamics -> controller feedback
%     SihControlCmdBus - OUTPUT from M4
%
%   and saves them to M4_BusObjects.mat for M6 to load.
%
%   WHY THIS MATTERS: M3 warned that NumWaypoints and SeqNum are uint32.
%   If a Bus Object declares them as double, the model fails at COMPILE
%   time with a clear type error - annoying but cheap. The expensive
%   version of this bug is the one that compiles: uint32 arithmetic in
%   your own code saturates instead of going negative, so idx-1 at idx=0
%   silently gives 0. Declaring the types correctly here is what makes the
%   cheap failure happen instead of the expensive one.

MAXWP = 5000;                       % locked array height, per M3's contract

%% ---- SihPlanBus : M3 -> M4 -------------------------------------------
e = Simulink.BusElement.empty;
e(1) = mkElem('Waypoints',           [MAXWP 4], 'double');
e(2) = mkElem('NumWaypoints',        1,         'uint32');
e(3) = mkElem('Directions',          [MAXWP 1], 'double');
e(4) = mkElem('SeqNum',              1,         'uint32');
e(5) = mkElem('GenerationTimestamp', 1,         'double');
e(6) = mkElem('MapTimestamp',        1,         'double');
e(7) = mkElem('MapAgeAtPlan_s',      1,         'double');

SihPlanBus = Simulink.Bus;
SihPlanBus.Elements    = e;
SihPlanBus.Description = ['Plan from M3 (Hybrid A* + velocity profiler). ' ...
    'Waypoints cols = [x y theta v_signed], world frame. Rows beyond ' ...
    'NumWaypoints are ZERO PADDING - always gate reads on NumWaypoints. ' ...
    'Timestamps are on navClock.m, a session-local monotonic clock, NOT ' ...
    'wall-clock. Do not compare against posixtime or datetime.'];

%% ---- SihEgoStateBus : vehicle dynamics -> controller -----------------
e = Simulink.BusElement.empty;
e(1) = mkElem('X',   1, 'double');      % rear-axle position, world frame
e(2) = mkElem('Y',   1, 'double');
e(3) = mkElem('psi', 1, 'double');      % heading [rad], CCW positive
e(4) = mkElem('v',   1, 'double');      % SIGNED speed [m/s]

SihEgoStateBus = Simulink.Bus;
SihEgoStateBus.Elements    = e;
SihEgoStateBus.Description = ['Ego state at the REAR AXLE, world frame, ' ...
    'ISO 8855 sign conventions (x forward, y left, theta CCW).'];

%% ---- SihControlCmdBus : M4 -> plant AND back to M3 -------------------
e = Simulink.BusElement.empty;
e(1) = mkElem('steering_angle', 1, 'double');   % [rad], front wheel
e(2) = mkElem('acceleration',   1, 'double');   % [m/s^2], longitudinal
e(3) = mkElem('speedCap_mps',   1, 'double');   % -> M3 assignVelocityProfile

SihControlCmdBus = Simulink.Bus;
SihControlCmdBus.Elements    = e;
SihControlCmdBus.Description = ['Control output. NOTE the third field: ' ...
    'speedCap_mps is NOT an actuator command. It is derived from ' ...
    'driving_mode and feeds BACK UPSTREAM to M3 as assignVelocityProfile''s ' ...
    'externalSpeedCap_mps argument. This edge is not on the original ' ...
    'architecture diagram - M6 must add it.'];

%% ---- Stub output value for the Day-1 placeholder ---------------------
% Simulink.Bus.createMATLABStruct builds a zero-initialised struct whose
% fields and types match the bus exactly. Feeding THAT to a Constant block
% is far less error-prone than wiring three Constants into a Bus Creator
% and hand-naming the signal lines to match element names.
assignin('base', 'SihControlCmdBus', SihControlCmdBus);
m4StubCmd.steering_angle = 0;
m4StubCmd.acceleration   = 0;
m4StubCmd.speedCap_mps   = 8.33;      % CRUISE cap, so the stub is not a STOP

%% ---- Publish ---------------------------------------------------------
assignin('base', 'm4StubCmd',        m4StubCmd);
assignin('base', 'SihPlanBus',       SihPlanBus);
assignin('base', 'SihEgoStateBus',   SihEgoStateBus);
assignin('base', 'SihControlCmdBus', SihControlCmdBus);

save('M4_BusObjects.mat', 'SihPlanBus', 'SihEgoStateBus', 'SihControlCmdBus');

fprintf('M4 bus objects created and saved to M4_BusObjects.mat\n');
fprintf('  SihPlanBus       : %d elements\n', numel(SihPlanBus.Elements));
fprintf('  SihEgoStateBus   : %d elements\n', numel(SihEgoStateBus.Elements));
fprintf('  SihControlCmdBus : %d elements\n', numel(SihControlCmdBus.Elements));
fprintf('  DrivingMode enum : %s\n', strjoin(string(enumeration('DrivingMode'))', ', '));
fprintf('  m4StubCmd        : stub output struct ready for the Constant block\n');
end

% -----------------------------------------------------------------------
function be = mkElem(name, dims, dtype)
be            = Simulink.BusElement;
be.Name       = name;
be.Dimensions = dims;
be.DataType   = dtype;
be.Complexity = 'real';
end
