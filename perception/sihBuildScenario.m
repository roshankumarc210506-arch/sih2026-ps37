function [scenario, egoVehicle, classOf] = sihBuildScenario(cfg)
%SIHBUILDSCENARIO  Mixed-traffic unmarked-road scenario for perception testing.
%
%   [scenario, ego, classOf] = sihBuildScenario(sihConfig());
%
%   This is M1's OWN test bench — it exists so you are not blocked waiting
%   for M5's RoadRunner scenes. On Day 2 you swap this out for M5's
%   scenario file; the rest of the pipeline is unchanged.
%
%   Contains all 6 classes: car, two-wheeler, auto-rickshaw, pushcart,
%   pedestrian, animal — including deliberate occlusion and a close-crossing
%   event, because those are exactly what breaks track association.
%
%   classOf : containers.Map from ActorID -> AgentClass (ground truth)

if nargin < 1, cfg = sihConfig(); end

scenario = drivingScenario('SampleTime', cfg.SampleTime, 'StopTime', cfg.StopTime);

% Unmarked 7 m village road, gentle curve
roadCenters = [  0  0  0
                60  0  0
               120  8  0
               180 12  0];
road(scenario, roadCenters, 7, 'Name', 'UnmarkedVillageRoad');

classOf = containers.Map('KeyType','double','ValueType','any');

% ---------------- Ego ----------------
egoVehicle = vehicle(scenario, ...
    'ClassID',      sihScenarioClassID(AgentClass.Car), ...
    'Name',         'Ego', ...
    'Length',       4.7, ...
    'Width',        1.8, ...
    'Height',       1.5, ...
    'Wheelbase',    2.5, ...
    'RearOverhang', 0.9, ...
    'Mesh',         sihActorMesh(AgentClass.Car, [4.7 1.8 1.5]), ...
    'Position',     [0 -1.5 0]);
smoothTrajectory(egoVehicle, roadCenters + [0 -1.5 0], 8);
% Day 4 DECISION: stays scripted, NOT swapped for M4's real ego trajectory.
% M4's real trajectory only covers X:0-55.5m; this scenario's other 6
% actors are choreographed across X:10-175m (see their smoothTrajectory
% calls below) -- swapping would mean 5 of 7 actors never come near ego
% at all, gutting the occlusion/close-crossing stress-testing this
% scenario exists for. Also: this scenario is explicitly temporary (see
% file header -- "swap this out for M5's scenario file"), and wiring the
% ego actor to be pose-driven externally is already M5's own Phase 1
% task for her real scenario files (task-split doc), not something to
% bolt onto this throwaway test bench. Every Day 4 result (ground
% filter, dedup, tracker tuning) was measured against THIS scripted
% path -- changing it now would invalidate that baseline for a scenario
% about to be replaced anyway.

% ---------------- 1. Oncoming car ----------------
a = vehicle(scenario, 'ClassID', sihScenarioClassID(AgentClass.Car), 'Name','OncomingCar', ...
    'Length',4.5,'Width',1.8,'Height',1.5, ...
    'Mesh',sihActorMesh(AgentClass.Car, [4.5 1.8 1.5]),'Position',[150 2 0]);
smoothTrajectory(a, [150 2 0; 100 2 0; 55 1.5 0; 10 1.5 0], 7);
classOf(a.ActorID) = AgentClass.Car;

% ---------------- 2. Two-wheeler weaving ahead of ego ----------------
a = actor(scenario, 'ClassID', sihScenarioClassID(AgentClass.TwoWheeler), 'Name','TwoWheeler', ...
    'Length',1.9,'Width',0.7,'Height',1.4, ...
    'Mesh',sihActorMesh(AgentClass.TwoWheeler, [1.9 0.7 1.4]),'Position',[25 -2.5 0]);
smoothTrajectory(a, [25 -2.5 0; 45 -0.5 0; 65 -2.6 0; 90 -0.8 0; 120 6 0; 160 10 0], 9);
classOf(a.ActorID) = AgentClass.TwoWheeler;

% ---------------- 3. Slow auto-rickshaw directly ahead (occluder) ----------
a = vehicle(scenario, 'ClassID', sihScenarioClassID(AgentClass.AutoRickshaw), 'Name','AutoRickshaw', ...
    'Length',2.6,'Width',1.4,'Height',1.7,'Wheelbase',2.0, ...
    'Mesh',sihActorMesh(AgentClass.AutoRickshaw, [2.6 1.4 1.7]),'Position',[38 -1.4 0]);
smoothTrajectory(a, [38 -1.4 0; 80 -1.4 0; 130 6.5 0; 175 10.5 0], 5.5);
classOf(a.ActorID) = AgentClass.AutoRickshaw;

% ---------------- 4. Pushcart at the road edge ----------------
a = actor(scenario, 'ClassID', sihScenarioClassID(AgentClass.PushCart), 'Name','PushCart', ...
    'Length',1.6,'Width',0.9,'Height',1.1, ...
    'Mesh',sihActorMesh(AgentClass.PushCart, [1.6 0.9 1.1]),'Position',[70 -3.2 0]);
smoothTrajectory(a, [70 -3.2 0; 84 -3.3 0], 1.1);
classOf(a.ActorID) = AgentClass.PushCart;

% ---------------- 5. Pedestrian crossing ----------------
a = actor(scenario, 'ClassID', sihScenarioClassID(AgentClass.Pedestrian), 'Name','Pedestrian', ...
    'Length',0.5,'Width',0.6,'Height',1.7, ...
    'Mesh',sihActorMesh(AgentClass.Pedestrian, [0.5 0.6 1.7]),'Position',[96 -5 0]);
smoothTrajectory(a, [96 -5 0; 98 -1 0; 100 4 0; 101 7 0], 1.4);
classOf(a.ActorID) = AgentClass.Pedestrian;

% ---------------- 6. Cattle wandering into the road (the hard one) ------
a = actor(scenario, 'ClassID', sihScenarioClassID(AgentClass.Animal), 'Name','Cow', ...
    'Length',2.2,'Width',0.9,'Height',1.5, ...
    'Mesh',sihActorMesh(AgentClass.Animal, [2.2 0.9 1.5]),'Position',[135 -6 0]);
smoothTrajectory(a, [135 -6 0; 137 -2 0; 139 3 0; 141 6 0], 1.3);
classOf(a.ActorID) = AgentClass.Animal;

% ---------------- 7. Second pedestrian, close-crossing with #5 ----------
% Two pedestrians crossing near each other is the classic JPDA stress test.
a = actor(scenario, 'ClassID', sihScenarioClassID(AgentClass.Pedestrian), 'Name','Pedestrian2', ...
    'Length',0.5,'Width',0.6,'Height',1.7, ...
    'Mesh',sihActorMesh(AgentClass.Pedestrian, [0.5 0.6 1.7]),'Position',[101 6 0]);
smoothTrajectory(a, [101 6 0; 99 1 0; 97 -4 0], 1.5);
classOf(a.ActorID) = AgentClass.Pedestrian;

fprintf('[M1] Scenario built: %d target actors, %.1f s, dt=%.2f s\n', ...
    classOf.Count, cfg.StopTime, cfg.SampleTime);
end

% ------------------------------------------------------------------------
function id = sihScenarioClassID(agentClass)
% AgentClass -> nearest valid drivingScenario ClassID.
% These are UNRELATED enums. vehicle(): 1=Car, 2=Truck.
% actor(): 3=Bicycle, 4=Pedestrian, 5=JerseyBarrier, 6=Guardrail.
switch agentClass
    case AgentClass.Car,          id = 1;
    case AgentClass.AutoRickshaw, id = 1;
    case AgentClass.TwoWheeler,   id = 3;
    case AgentClass.PushCart,     id = 4;
    case AgentClass.Pedestrian,   id = 4;
    case AgentClass.Animal,       id = 4;
    otherwise,                    id = 4;
end
end
