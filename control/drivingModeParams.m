function m = drivingModeParams(mode)
%DRIVINGMODEPARAMS Maps a DrivingMode enum to concrete controller numbers.
%
%   THIS FILE IS THE MISSING PIECE OF THE CONTRACT. The locked contract says
%   driving_mode "tightens MPC constraints + caps velocity profiler speed
%   limit" but never says by how much. This defines it.
%
%   Consumers:
%     M4 (here)  - obstacleMargin_m, aMin/aMax, weight scaling
%     M3         - speedCap_mps, passed as assignVelocityProfile's 3rd arg
%                  (externalSpeedCap_mps)
%     M5         - needs to know these exist so chart thresholds make sense
%
%   IMPORTANT - the speed cap is applied TWICE, deliberately:
%     (1) upstream, by M3's velocity profiler, so the whole plan is shaped
%         to the cap and the geometry stays consistent;
%     (2) locally, by M4, clamping the reference before it enters the NMPC.
%   Without (2), a STOP command would take a full replan cycle (~250-500 ms)
%   to take effect. That is not acceptable for a STOP. Do not remove the
%   local clamp as "redundant" - it is the fast path.

arguments
    mode (1,1) DrivingMode
end

switch mode

    case DrivingMode.CRUISE
        m.speedCap_mps     = 8.33;   % 30 km/h - nominal unstructured-road speed
        m.obstacleMargin_m = 0.50;   % added to disc radius in soft constraint
        m.aMax             =  2.0;
        m.aMin             = -3.0;
        m.jerkLimit        =  1.0;   % |da| per sample
        m.wDeviationScale  =  1.0;   % multiplier on path-deviation weight
        m.wEffortScale     =  1.0;

    case DrivingMode.CAUTIOUS
        m.speedCap_mps     = 4.17;   % 15 km/h
        m.obstacleMargin_m = 1.00;
        m.aMax             =  1.0;   % gentler acceleration, harder braking
        m.aMin             = -3.5;
        m.jerkLimit        =  0.8;
        m.wDeviationScale  =  0.7;   % allow more deviation to dodge agents
        m.wEffortScale     =  1.0;

    case DrivingMode.YIELD
        m.speedCap_mps     = 1.39;   % 5 km/h - creeping
        m.obstacleMargin_m = 1.50;
        m.aMax             =  0.5;
        m.aMin             = -4.0;
        m.jerkLimit        =  1.2;   % permit sharper braking when yielding
        m.wDeviationScale  =  0.5;
        m.wEffortScale     =  0.8;

    case DrivingMode.STOP
        m.speedCap_mps     = 0.0;
        m.obstacleMargin_m = 2.00;
        m.aMax             =  0.0;   % cannot command positive accel at all
        m.aMin             = -5.0;
        m.jerkLimit        =  2.0;
        m.wDeviationScale  =  0.2;   % stopping beats staying on the line
        m.wEffortScale     =  0.5;

    otherwise
        error('drivingModeParams:unknownMode', 'Unhandled DrivingMode.');
end

m.mode = mode;

%% ---- Note for M5 on hysteresis --------------------------------------
% These numbers assume the mode does not flicker. A CRUISE<->YIELD
% oscillation at chart rate produces a speed cap that jumps 8.33 -> 1.39
% every few samples, which the NMPC will turn into visible surging.
% Suggested guards on your side:
%   - minimum dwell time of ~0.5 s in any mode before a downgrade
%     (upgrades to a MORE severe mode should be immediate - no dwell)
%   - separate enter/exit thresholds on agent-density and risk-zone guards
end
