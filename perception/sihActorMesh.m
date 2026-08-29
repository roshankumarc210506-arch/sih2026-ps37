function mesh = sihActorMesh(agentClass, dims)
%SIHACTORMESH  Scaled 3D mesh for LiDAR ray tracing, per AgentClass.
%
%   mesh = sihActorMesh(AgentClass.Car, [4.7 1.8 1.5])
%   mesh = sihActorMesh(AgentClass.PushCart, [1.6 0.9 1.1])
%
%   dims = [Length, Width, Height] in metres — pass the SAME values used
%   to create the actor (vehicle()/actor() Length/Width/Height), so the
%   ray-traced geometry matches the declared bounding box exactly. This
%   is deliberate: MathWorks' six stock meshes (carMesh, truckMesh,
%   pedestrianMesh, bicycleMesh, guardrailMesh, jerseyBarrierMesh) are
%   ALL zero-argument functions returning a FIXED default size —
%   carMesh() is always 4.7x1.8x1.4m regardless of what's in the
%   scenario. Using a mesh without scaleToFit-ing it to the actor's own
%   dimensions would silently decouple ray-traced geometry from declared
%   size — exactly the class of bug this project has hit repeatedly
%   today (dimension drift between two independently-set sources).
%
%   Coverage, confirmed via direct inventory of driving.scenario.*Mesh:
%     Car          -> carMesh(), scaled        (direct match)
%     TwoWheeler   -> bicycleMesh(), scaled     (reasonable stand-in)
%     Pedestrian   -> pedestrianMesh(), scaled  (direct match)
%     AutoRickshaw -> cuboid, scaled            (no stock mesh exists)
%     PushCart     -> cuboid, scaled            (no stock mesh exists)
%     Animal       -> cuboid, scaled            (no stock mesh exists;
%                     a plain box is a pragmatic simplification for
%                     ray-tracing/clustering purposes — good enough to
%                     get a point cloud with roughly correct extent,
%                     not attempting to model legs/head. Revisit only if
%                     Animal-class LiDAR clustering specifically turns
%                     out to need better silhouette fidelity.)
%
%   Scope note: road surface mesh is NOT handled here. M3's
%   roadBoundaries() already covers static road-edge geometry via a
%   separate mechanism; this function is only for dynamic/static AGENTS
%   that need to appear in the LiDAR point cloud for clustering into
%   tracked detections.

switch agentClass
    case AgentClass.Car
        base = driving.scenario.carMesh;
    case AgentClass.TwoWheeler
        base = driving.scenario.bicycleMesh;
    case AgentClass.Pedestrian
        base = driving.scenario.pedestrianMesh;
    otherwise
        % AutoRickshaw, PushCart, Animal, Unknown fallback: plain cuboid.
        base = extendedObjectMesh('cuboid');
end

% scaleToFit needs a struct with Length/Width/Height/OriginOffset fields
% (or an undocumented raw 6-elt [L W H offX offY offZ] array) -- a plain
% 3-elt [L W H] matches neither and errors with "numel 6" deep inside
% extendedObjectMesh/scaleToFit. No offset: mesh stays centered on the
% actor's own local origin, same as drivingScenario already assumes.
fitDims = struct('Length', dims(1), 'Width', dims(2), 'Height', dims(3), ...
                  'OriginOffset', [0 0 0]);
mesh = scaleToFit(base, fitDims);
end
