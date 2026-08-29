%% RUNPLANNERDEMO  End-to-end: config -> map -> plan -> velocity -> bus -> audit
clear; clc; close all;

cfg = planner_config();
[omap, rawmap, mapInfo] = buildPlaceholderMap(cfg);
gp  = planGlobalPath(omap, cfg, mapInfo);

if ~gp.IsPathFound
    error('runPlannerDemo:NoPath', 'No path found. Check start/goal and map.');
end

vp  = assignVelocityProfile(gp, cfg);
bus = packPlanBus(vp, gp, cfg, true);

if ~exist('checkPathFootprint','file')
    warning('runPlannerDemo:NoAuditFn', 'checkPathFootprint.m not found - skipping audit.');
    audit = [];
else
    if exist('bwdist','file') ~= 2
        warning('runPlannerDemo:NoIPT', ...
            'Image Processing Toolbox (bwdist) not found - skipping footprint audit.');
        audit = [];
    else
        audit = checkPathFootprint(gp, rawmap, cfg);
    end
end

%% ---------- report ----------
fprintf('\n================ PLAN SUMMARY ================\n');
fprintf('Waypoints           : %d\n',       gp.NumWaypoints);
fprintf('Path length         : %.2f m\n',   gp.PathLength_m);
fprintf('Reverse length      : %.2f m\n',   gp.ReverseLength_m);
fprintf('Cusps               : %d\n',       gp.NumDirectionSwitches);
fprintf('Search mode used    : %s\n',       gp.SearchModeUsed);
fprintf('Planning time       : %.3f s\n',   gp.PlanningTime_s);
fprintf('Map age at plan     : %.3f s  (budget %.2f s)\n', gp.MapAgeAtPlan_s, cfg.Bus.MaxMapAge_s);
fprintf('Dir disagreements   : %d  <- want 0\n', gp.DirectionDisagreements);

fprintf('\n================ VELOCITY SUMMARY ================\n');
fprintf('Max speed reached   : %.2f m/s (cap %.2f)\n', vp.MaxSpeedReached_mps, vp.SpeedCapApplied_mps);
fprintf('Total time          : %.2f s\n', vp.TotalTime_s);
fprintf('v at start/goal     : %.3f / %.3f  (want 0 / 0)\n', vp.Waypoints(1,4), vp.Waypoints(end,4));

fprintf('\n================ BUS SUMMARY ================\n');
fprintf('SeqNum              : %d\n', bus.SeqNum);
fprintf('NumWaypoints        : %d / %d slots\n', bus.NumWaypoints, size(bus.Waypoints,1));
if bus.MapAgeAtPlan_s <= cfg.Bus.MaxMapAge_s
    ageStatus = 'OK';
else
    ageStatus = 'STALE - would be rejected by M4';
end
fprintf('MapAgeAtPlan_s       : %.3f s  --> %s\n', bus.MapAgeAtPlan_s, ageStatus);
if ~isempty(audit)
    fprintf('\n================ FOOTPRINT AUDIT ================\n');
    fprintf('Mode used by planner        : %s\n', audit.FootprintMode);
    fprintf('Planner-model min clearance : %.3f m  (at waypoint %d)\n', ...
        audit.MinClearance_m, audit.MinClearanceWaypointIdx);
    fprintf('  waypoints w/ negative clr : %d / %d\n', audit.NumWaypointsNegative, gp.NumWaypoints);
    fprintf('TRUE BODY min clearance     : %.3f m  (at waypoint %d)\n', ...
        audit.TrueBodyMinClearance_m, audit.TrueBodyWorstIdx);
    fprintf('  TRUE BODY collisions      : %d / %d waypoints  <-- the number that matters\n', ...
        audit.TrueBodyNumCollisions, gp.NumWaypoints);
    if audit.TrueBodyNumCollisions > 0
        fprintf('  *** planner-model clearance says the path is safe, but the actual\n');
        fprintf('      4.7x1.8 m body clips an obstacle. This is the 2.52 m disc\n');
        fprintf('      under-coverage (1.385 m at the nose) becoming real on this map. ***\n');
    end
end

%% ---------- plots ----------
figure('Name','Full pipeline result','Color','w','Position',[80 80 1300 800]);

subplot(2,2,1);
show(omap); hold on;
Sst = gp.States; fwd = gp.Directions > 0;
plot(Sst(fwd,1), Sst(fwd,2), 'b.', 'MarkerSize',7);
plot(Sst(~fwd,1),Sst(~fwd,2),'m.', 'MarkerSize',7);
plot(gp.StartPose(1),gp.StartPose(2),'go','MarkerFaceColor','g','MarkerSize',9);
plot(gp.GoalPose(1), gp.GoalPose(2), 'ro','MarkerFaceColor','r','MarkerSize',9);
title(sprintf('Path: %.1fm, %d cusps',gp.PathLength_m,gp.NumDirectionSwitches));
axis equal tight; hold off;

subplot(2,2,2);
plot(vp.ArcLength_m, vp.Waypoints(:,4), 'b-','LineWidth',1.5); grid on; yline(0,'k:');
xlabel('s [m]'); ylabel('v [m/s]'); title('Velocity profile');

subplot(2,2,3);
if ~isempty(audit)
    plot(gp.States(:,1)*0 + (1:gp.NumWaypoints), audit.PerWaypointClearance_m, 'b-', 'LineWidth',1.2); hold on;
    plot(1:gp.NumWaypoints, audit.TrueBodyClearance_m, 'r-', 'LineWidth',1.2);
    yline(0,'k--');
    legend('planner-model clearance','TRUE body clearance','Location','best');
    xlabel('waypoint index'); ylabel('clearance [m]');
    title('Clearance: planner model vs true body');
else
    text(0.5,0.5,'Audit unavailable (no Image Processing Toolbox)','HorizontalAlignment','center');
    axis off;
end

subplot(2,2,4);
show(rawmap); hold on;
plot(Sst(:,1), Sst(:,2), 'b-', 'LineWidth', 1.5);
if ~isempty(audit) && audit.TrueBodyNumCollisions > 0
    badIdx = find(audit.TrueBodyClearance_m < 0);
    plot(Sst(badIdx,1), Sst(badIdx,2), 'rx', 'MarkerSize',8,'LineWidth',1.5);
end
title('Raw obstacles + path (red x = true-body collision)');
axis equal tight; hold off;

