function [accept, guardState, reason] = planFreshnessGuard(plan, guardState, nowSec, cfg)
%PLANFRESHNESSGUARD Decide whether to accept an incoming plan bus.
%
%   Three independent failure modes, three independent checks. Do not
%   collapse them into one - they need different responses.
%
%     A. SUPERSEDED : SeqNum <= last accepted. An old plan arrived late or
%                     the bus is holding a value. Ignore, keep tracking the
%                     plan we have. This is normal, not an error.
%
%     B. STALE MAP  : MapAgeAtPlan_s too large. M3 planned against an
%                     occupancy map that was already old at plan time. The
%                     geometry may route through an obstacle that has since
%                     appeared. Accept but DEGRADE - request a more severe
%                     driving mode rather than blindly following.
%
%     C. STALE PLAN : nowSec - GenerationTimestamp too large. The plan is
%                     fine but nothing new has arrived - M3 may have hung
%                     or is taking too long. Coast on the existing plan, and
%                     escalate to STOP past a hard limit.
%
%   INPUTS
%     plan       - plan bus struct
%     guardState - struct('lastSeq', uint32, 'lastAcceptSec', double)
%     nowSec     - current time on the SAME clock as GenerationTimestamp
%                  (navClock.m, session-local monotonic - NOT wall clock,
%                   NOT posixtime. Mixing clocks here silently produces
%                   ages of ~1.7e9 seconds and everything hard-stops.)
%     cfg        - struct with fields:
%                    maxMapAge_s       (e.g. 0.50)
%                    maxPlanAge_s      (e.g. 1.00)  -> degrade
%                    hardPlanAge_s     (e.g. 2.00)  -> force STOP
%
%   OUTPUTS
%     accept     - true if this plan should replace the current reference
%     guardState - updated
%     reason     - struct of booleans + a suggested minimum DrivingMode

reason = struct('superseded', false, 'staleMap', false, 'stalePlan', false, ...
                'hardStale', false, 'suggestedMinMode', DrivingMode.CRUISE);

seq = double(plan.SeqNum);                  % uint32 -> double before compare

%% ---- A. Superseded / duplicate ---------------------------------------
if seq <= double(guardState.lastSeq)
    accept = false;
    reason.superseded = true;
else
    accept = true;
end

%% ---- B. Map staleness at plan time -----------------------------------
if plan.MapAgeAtPlan_s > cfg.maxMapAge_s
    reason.staleMap = true;
    reason.suggestedMinMode = DrivingMode.CAUTIOUS;
end

%% ---- C. Plan age right now -------------------------------------------
planAge = nowSec - plan.GenerationTimestamp;

if planAge > cfg.hardPlanAge_s
    reason.stalePlan = true;
    reason.hardStale = true;
    reason.suggestedMinMode = DrivingMode.STOP;
elseif planAge > cfg.maxPlanAge_s
    reason.stalePlan = true;
    if reason.suggestedMinMode < DrivingMode.YIELD
        reason.suggestedMinMode = DrivingMode.YIELD;
    end
end

%% ---- Commit ----------------------------------------------------------
if accept
    guardState.lastSeq       = plan.SeqNum;
    guardState.lastAcceptSec = nowSec;
end

end
