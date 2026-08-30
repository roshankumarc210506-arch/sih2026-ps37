function createM4BusObjects()
%CREATEM4BUSOBJECTS Define Simulink.Bus objects on M4's boundary.
%
%   OWNS:  SihControlCmdBus (M4 output bus)
%   OWNS:  SihPlanBus (M4-side definition, consumed from M3's plan;
%          see sihDefineBuses.m for why this lives here)
%   DOES NOT OWN: SihEgoBus (defined in buses/sihCreateBuses.m by M1)
%   DELETED: SihEgoStateBus - was a duplicate of SihEgoBus, removed.

MAX_WAYPOINTS = 5000;   % CONFIRMED by M3, Day 1 - matches cfg.Bus.MaxWaypoints
                         % and cfg.Plan.MaxNumPathStates in planner_config.m

%% ---- Build SihControlCmdBus FIRST -----------------------------------
% Must exist in workspace BEFORE createMATLABStruct is called below.
e(1) = mkElem('steering_angle', 1, 'double');
e(2) = mkElem('acceleration',   1, 'double');
e(3) = mkElem('speedCap_mps',   1, 'double');

SihControlCmdBus             = Simulink.Bus;
SihControlCmdBus.Elements    = e;
SihControlCmdBus.Description = ['Control output. steering_angle [rad] and ' ...
    'acceleration [m/s^2] go to the plant. speedCap_mps [m/s] routes back ' ...
    'to M3 velocity profiler as externalSpeedCap_mps (feedback edge, not ' ...
    'yet wired in top model - M6 to add when GlobalPlanner goes real).'];

%% ---- Publish to workspace BEFORE createMATLABStruct call ------------
% createMATLABStruct looks up the bus by name in the BASE workspace.
% The assignin must come BEFORE that call, not after.
assignin('base', 'SihControlCmdBus', SihControlCmdBus);

%% ---- Build SihPlanBus -------------------------------------------------
% Contract locked per Section 6 / sihDefineBuses.m comments:
%   Waypoints            double  [MAX_WAYPOINTS x 4]  cols=[x,y,theta,v_signed]
%   NumWaypoints          uint32  1   GATE ALL READS ON THIS
%   Directions            double  [MAX_WAYPOINTS x 1]  +1=forward,-1=reverse
%   SeqNum                 uint32  1   increments every replan
%   GenerationTimestamp    double  1   navClock (session-local monotonic)
%   MapTimestamp           double  1
%   MapAgeAtPlan_s         double  1
p(1) = mkElem('Waypoints',           [MAX_WAYPOINTS 4], 'double');
p(2) = mkElem('NumWaypoints',        1,                 'uint32');
p(3) = mkElem('Directions',          [MAX_WAYPOINTS 1], 'double');
p(4) = mkElem('SeqNum',              1,                 'uint32');
p(5) = mkElem('GenerationTimestamp', 1,                 'double');
p(6) = mkElem('MapTimestamp',        1,                 'double');
p(7) = mkElem('MapAgeAtPlan_s',      1,                 'double');
p(8) = mkElem('PlannerInfeasible', 1, 'boolean');

SihPlanBus             = Simulink.Bus;
SihPlanBus.Elements    = p;
SihPlanBus.Description = ['Plan bus from M3. Rows beyond NumWaypoints are ' ...
    'zero padding at the map origin - always gate reads on NumWaypoints, ' ...
    'never numel(Waypoints). Directions is double (cast from planner''s ' ...
    'native int8 in assignVelocityProfile.m / packPlanBus.m).'];

assignin('base', 'SihPlanBus', SihPlanBus);

%% ---- Now safe to create the stub struct -----------------------------
m4StubCmd                = Simulink.Bus.createMATLABStruct('SihControlCmdBus');
m4StubCmd.steering_angle = 0;
m4StubCmd.acceleration   = 0;
m4StubCmd.speedCap_mps   = 8.33;   % CRUISE cap - stub is not a STOP

assignin('base', 'm4StubCmd', m4StubCmd);

%% ---- Save and report ------------------------------------------------
save('M4_BusObjects.mat', 'SihControlCmdBus', 'SihPlanBus');

fprintf('M4 bus objects created\n');
fprintf('  SihControlCmdBus : %d elements\n', numel(SihControlCmdBus.Elements));
fprintf('  SihPlanBus       : %d elements (MAX_WAYPOINTS=%d)\n', ...
        numel(SihPlanBus.Elements), MAX_WAYPOINTS);
fprintf('  SihEgoStateBus   : DELETED (was duplicate of SihEgoBus)\n');
fprintf('  SihEgoBus        : owned by M1, loaded via sihCreateBuses.m\n');
fprintf('  DrivingMode enum : %s\n', ...
        strjoin(string(enumeration('DrivingMode'))', ', '));
fprintf('  m4StubCmd        : stub output struct ready\n');
end

function be = mkElem(name, dims, dtype)
be            = Simulink.BusElement;
be.Name       = name;
be.Dimensions = dims;
be.DataType   = dtype;
be.Complexity = 'real';
end
