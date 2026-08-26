function sihDefineBuses()
%SIHDEFINEBUSES  Entry point for all SIH PS37 bus definitions.
%   Call once per model InitFcn. Perception buses are owned by M1
%   (sihCreateBuses); this adds buses other subsystems need.
%   PLACEHOLDER — integration lead to review and extend.

cfg = sihConfig();
sihCreateBuses(cfg);   % SihTrackBus, SihPerceptionBus, SihEgoBus

% TODO: prediction bus, plan bus, driving-mode bus, control bus go
% here. Do NOT redefine SihTrackBus/SihPerceptionBus/SihEgoBus in
% this function — sihCreateBuses() above is the single source of
% truth for those three.
end
