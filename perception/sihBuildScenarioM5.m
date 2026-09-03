function [scenario, egoVehicle, classOf] = sihBuildScenarioM5(scenarioName, cfg)
%SIHBUILDSCENARIOM5  Wrap M5's 5 scenario builders for use in runPerceptionStub.
%
%   [scenario, egoVehicle, classOf] = sihBuildScenarioM5('villageRoad', cfg);
%
%   scenarioName: 'villageRoad' | 'urbanIntersection' | 'highwayMerge'
%                 'denseMarket' | 'cattleCrossing'
%
%   Matches the [scenario, egoVehicle, classOf] signature of sihBuildScenario.m
%   so runPerceptionStub.m needs no changes -- swap the call site only.
%
%   classOf: built from actor Name strings, since M5's builders use ADT
%   ClassIDs (not AgentClass). The name->class map below is the single
%   canonical definition. If M5 renames an actor, update here AND tell M1.
%
%   KNOWN LIMITATION: M5's builders don't attach sihActorMesh() to actors.
%   Real LiDAR ray-tracing in sihCreateLidar uses actor meshes for occlusion.
%   Without meshes, lidar detections fall back to bounding-box approximation.
%   This is acceptable for Phase 2 integration runs -- not the same fidelity
%   as Phase 1's carefully meshed actors, so don't directly compare RMSE
%   numbers directly against Phase 1 baselines.
%
%   MergingTruck1 is mapped to AgentClass.Unknown (not Car): ClassID=2 is
%   ADT Truck, which has no equivalent in the AgentClass enum. Unknown gets
%   the most conservative costmap inflation (0.90m, same as Animal) -- safer
%   than Car's 0.30m for a large merging vehicle. Long-term fix is adding a
%   proper Truck class to the enum; deferred, not dropped.

if nargin < 2, cfg = sihConfig(); end

% ---- Build the raw scenario from M5's builder ----
switch lower(scenarioName)
    case 'villageroad'
        scenario = buildVillageRoadScenario();
    case 'urbanintersection'
        scenario = buildUrbanIntersectionScenario();
    case 'highwaymerge'
        scenario = buildHighwayMergeScenario();
    case 'densemarket'
        scenario = buildDenseMarketScenario();
    case 'cattlecrossing'
        scenario = buildCattleCrossingScenario();
    otherwise
        error('sihBuildScenarioM5:unknownScenario', ...
              ['Unknown scenario "%s". Valid: villageRoad, urbanIntersection, ' ...
               'highwayMerge, denseMarket, cattleCrossing'], scenarioName);
end

% ---- Extract egoVehicle (ActorID==1 by convention, asserted in all builders) ----
egoVehicle = [];
for k = 1:numel(scenario.Actors)
    if scenario.Actors(k).ActorID == 1
        egoVehicle = scenario.Actors(k);
        break;
    end
end
assert(~isempty(egoVehicle), ...
    'sihBuildScenarioM5: no actor with ActorID==1 found -- addEgoVehicle-first convention broken.');

% ---- Build classOf from actor Name strings ----
% Single canonical name->AgentClass mapping for all 5 M5 scenarios.
% Names must match the Name field in M5's builder files exactly.
% If M5 renames an actor or adds one, update this map and notify M1.
classOf = containers.Map('KeyType','double','ValueType','any');

nameToClass = containers.Map( ...
    {'Pushcart1','Pushcart2','Pushcart3','Pushcart4', ...
     'TwoWheeler1','TwoWheeler2', ...
     'Pedestrian1','Pedestrian2','Pedestrian3','Pedestrian4', ...
     'Animal1','Cattle1', ...
     'CrossCar1','LeadCar1','AutoRickshaw1', ...
     'MergingTruck1'}, ...
    {AgentClass.PushCart,    AgentClass.PushCart,    AgentClass.PushCart,    AgentClass.PushCart, ...
     AgentClass.TwoWheeler,  AgentClass.TwoWheeler, ...
     AgentClass.Pedestrian,  AgentClass.Pedestrian,  AgentClass.Pedestrian,  AgentClass.Pedestrian, ...
     AgentClass.Animal,      AgentClass.Animal, ...
     AgentClass.Car,         AgentClass.Car,         AgentClass.AutoRickshaw, ...
     AgentClass.Unknown});   % Truck has no AgentClass equivalent -- see header

for k = 1:numel(scenario.Actors)
    act = scenario.Actors(k);
    if act.ActorID == 1, continue; end   % skip ego
    if isKey(nameToClass, act.Name)
        classOf(act.ActorID) = nameToClass(act.Name);
    else
        classOf(act.ActorID) = AgentClass.Unknown;
        warning('sihBuildScenarioM5:unknownActor', ...
                ['Actor "%s" (ID=%d) has no AgentClass mapping -- scored as Unknown. ' ...
                 'Add it to nameToClass if this is a real actor, not a placeholder.'], ...
                act.Name, act.ActorID);
    end
end

fprintf('[M1] M5 scenario "%s": %d target actors, %.1f s, dt=%.2f s\n', ...
    scenarioName, classOf.Count, scenario.StopTime, scenario.SampleTime);
end
