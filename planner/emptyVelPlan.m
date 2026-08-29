function velPlan = emptyVelPlan(planResult, cfg)
%EMPTYVELPLAN  Zero-waypoint velocity-profile struct, same field set as
%   assignVelocityProfile's normal output. Use this INSTEAD of calling
%   assignVelocityProfile when planResult.IsPathFound is false -
%   assignVelocityProfile's <2-waypoint guard is intentional and should
%   NOT be removed or bypassed; this is the correct alternate path.
velPlan = struct();
velPlan.Waypoints    = zeros(0,4);
velPlan.Directions   = zeros(0,1);
velPlan.Time_s       = zeros(0,1);
velPlan.Curvature    = zeros(0,1);
velPlan.ArcLength_m  = zeros(0,1);
velPlan.NumWaypoints = 0;
velPlan.CuspIndices  = zeros(0,1);
velPlan.MaxSpeedReached_mps = 0;
velPlan.TotalTime_s  = 0;
velPlan.SpeedCapApplied_mps = cfg.Vel.MaxSpeed_mps;
end