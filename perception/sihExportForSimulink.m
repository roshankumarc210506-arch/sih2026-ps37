function sihExportForSimulink(log, cfg, fileName)
%SIHEXPORTFORSIMULINK  Hand M2/M3/M6 real bus data on Day 1, zero risk.
%
%   sihExportForSimulink(log, cfg)
%   sihExportForSimulink(log, cfg, 'm1_perception_day1.mat')
%
%   WHY not a MATLAB Function block?
%     A MATLAB Function block requires code generation. trackerJPDA with a
%     containers.Map-based class voter is not codegen-clean, and burning
%     Day 1 fighting codegen errors is exactly how a 9-day timeline dies.
%
%   So: run perception in MATLAB, export a timestamped struct-of-arrays,
%   and let M6 drive it into the model with a From Workspace block feeding
%   the SihPerceptionBus. The rest of the team is unblocked TODAY.
%
%   Day 2 migration path, in order of preference:
%     1. Multi-Object Tracker Simulink block (ADT/SFTT library) — designed
%        for this, no codegen pain.
%     2. MATLAB System block (matlab.System subclass).
%     3. MATLAB Function block with coder.extrinsic on the tracker call.
%
%   Produces:
%     perceptionData : struct with .time and .signals.values (bus struct array)
%     egoData        : same, for the ego pose side bus

if nargin < 2 || isempty(cfg),      cfg      = sihConfig();               end
if nargin < 3 || isempty(fileName), fileName = 'm1_perception_day1.mat';  end

N = numel(log);
if N == 0
    error('sihExportForSimulink:emptyLog', 'Nothing to export — log is empty.');
end

time = zeros(N,1);
busVals(N,1) = struct('tracks', log(1).tracks, ...
                      'num_tracks', uint32(0), ...
                      'timestamp', 0);
egoVals(N,1) = struct('x',0,'y',0,'yaw',0,'velocity',0,'Timestamp',0);

for k = 1:N
    time(k)       = log(k).time;
    busVals(k)    = struct('tracks',     log(k).tracks, ...
                           'num_tracks', log(k).num_tracks, ...
                           'timestamp',  log(k).time);
    egoVals(k)    = log(k).ego;
end

perceptionData.time            = time;
perceptionData.signals.values  = busVals;
perceptionData.signals.dimensions = 1;

egoData.time                   = time;
egoData.signals.values         = egoVals;
egoData.signals.dimensions     = 1;

meta.frame        = cfg.Frame;
meta.sampleTime   = cfg.SampleTime;
meta.maxTracks    = cfg.MaxTracks;
meta.busName      = 'SihPerceptionBus';
meta.egoBusName   = 'SihEgoBus';
meta.generatedOn  = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
meta.note         = ['Loop 1:num_tracks. Slots beyond that have valid==false. ' ...
                     'covariance is 4x4 over [x y heading velocity], ego frame.'];

save(fileName, 'perceptionData', 'egoData', 'meta');

fprintf('[M1] Exported %d frames -> %s\n', N, fileName);
fprintf('     Give this to M6. Load it, then a From Workspace block typed as\n');
fprintf('     SihPerceptionBus drives the Perception subsystem output directly.\n');
end
