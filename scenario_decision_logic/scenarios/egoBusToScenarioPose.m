
function egoPose = egoBusToScenarioPose(actor_pose)
% ActorID=1 is hardcoded here, not derived dynamically (unlike M1's
% perception code, which correctly avoids this assumption -- see
% perception/sihCreateRealSensors.m). This is safe ONLY because
% addEgoVehicle(...) is called FIRST, before any other actor/vehicle,
% in all 5 scenario builders (buildVillageRoadScenario.m,
% buildUrbanIntersectionScenario.m, buildCattleCrossingScenario.m,
% buildDenseMarketScenario.m, buildHighwayMergeScenario.m) -- verified
% directly across all 5 files, Day 2. Driving Scenario Designer
% assigns ActorID in add-order, so this convention guarantees ego=1.
% IF THIS ORDERING EVER CHANGES in any scenario file (an actor added
% before addEgoVehicle), THIS LINE BREAKS SILENTLY -- it will
% mislabel a non-ego actor's pose as ego's. Do not remove/reorder
% addEgoVehicle calls without updating this to match M1's dynamic
% ActorID-matching pattern instead.
egoPose.ActorID = uint32(1);
egoPose.Position = [actor_pose.x, actor_pose.y, 0];
egoPose.Velocity = [actor_pose.velocity, 0, 0];
egoPose.Roll = 0;
egoPose.Pitch = 0;
egoPose.Yaw = actor_pose.yaw * 180/pi;
egoPose.AngularVelocity = [0, 0, 0];
end