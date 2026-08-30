function ccConfig = buildThreeCircleCollisionChecker(cfg)
%BUILDTHREECIRCLECOLLISIONCHECKER  InflationCollisionChecker matching the
%   EXACT three-circle geometry already locked in cfg.Foot (NumCircles,
%   CircleRadius_m, CircleOffsets_m - the SAME numbers checkPathFootprint.m
%   audits against).
%
%   Built EXPLICITLY via 'CenterPlacements'+'InflationRadius', NOT via
%   inflationCollisionChecker's own auto-derived default. If left
%   unspecified, the toolbox computes its own circle placement/radius to
%   enclose the vehicle - which is not guaranteed to match the numbers
%   already locked in cfg.Foot and already audited by checkPathFootprint.m.
%   Explicit construction means the live planner (this file, feeding
%   validatorVehicleCostmap) and the post-hoc audit can never silently
%   diverge onto two different three-circle geometries.
%
%   UNIT CONVERSION (verified, not assumed):
%   inflationCollisionChecker's CenterPlacements is normalized [0,1] along
%   the vehicle BODY length: 0 = rear bumper, 1 = front bumper. cfg.Foot's
%   CircleOffsets_m is meters from the REAR AXLE (this project's SE(2)
%   state reference point). Convert via:
%       placement = (offset_from_rear_axle_m + RearOverhang_m) / Length_m
%
%   Sanity-checked against cfg's actual locked numbers: offsets
%   [-0.1167, 1.4500, 3.0167] m with RearOverhang=0.9, Length=4.7 give
%   placements [0.1667, 0.5, 0.8333] - i.e. exactly 1/6, 1/2, 5/6. This
%   confirms cfg.Foot's existing geometry IS the canonical "divide the
%   body into NumCircles equal segments, one circle per segment center"
%   scheme (radius = half-diagonal of one segment), which is what this
%   function reproduces exactly.
%
%   Requires Automated Driving Toolbox (inflationCollisionChecker,
%   vehicleDimensions). Confirmed licensed for this project via
%   license('test','Automated_Driving_Toolbox') -> 1.

if ~strcmpi(cfg.Foot.Mode, 'threeCircle')
    error('buildThreeCircleCollisionChecker:WrongMode', ...
        ['cfg.Foot.Mode is ''%s'', not ''threeCircle''. This helper only ' ...
         'makes sense for three-circle mode.'], cfg.Foot.Mode);
end

% Height isn't used for 2-D footprint collision checking, but
% vehicleDimensions' constructor requires a value - nominal placeholder.
nominalHeight_m = 1.5;

vehicleDims = vehicleDimensions(cfg.Veh.Length_m, cfg.Veh.Width_m, nominalHeight_m, ...
    'Wheelbase',     cfg.Veh.Wheelbase_m, ...
    'FrontOverhang', cfg.Veh.FrontOverhang_m, ...
    'RearOverhang',  cfg.Veh.RearOverhang_m);

placements = (cfg.Foot.CircleOffsets_m + cfg.Veh.RearOverhang_m) / cfg.Veh.Length_m;

% If cfg.Foot.CircleOffsets_m and cfg.Veh dimensions ever drift out of
% sync with each other (e.g. someone changes Length_m without
% recomputing CircleOffsets_m), this catches it here rather than letting
% inflationCollisionChecker silently clip or error deeper in the stack.
assert(all(placements >= 0 & placements <= 1), ...
    'buildThreeCircleCollisionChecker:BadPlacement', ...
    ['Computed CenterPlacements %s fall outside [0,1]. cfg.Foot.CircleOffsets_m ' ...
     'is out of sync with cfg.Veh.Length_m/RearOverhang_m - re-derive one from the other.'], ...
    mat2str(placements));

ccConfig = inflationCollisionChecker(vehicleDims, ...
    'CenterPlacements', placements, ...
    'InflationRadius',  cfg.Foot.CircleRadius_m);
end