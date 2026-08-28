function t = navClock()
%NAVCLOCK  Monotonic seconds since first call. Session-local.
%
%   Placeholder for the Simulink simulation clock. Do NOT swap in
%   posixtime(datetime('now')) - a wall clock can step backwards on NTP
%   correction or DST, which corrupts MapAgeAtPlan_s silently.

persistent t0
if isempty(t0)
    t0 = tic;
end
t = toc(t0);
end