function [tracks, numTracks, egoPose, timestamp] = loadM1RealPerception(matFilePath, timeQuery)
%LOADM1REALPERCEPTION  Read one timestep of M1's REAL exported perception
%   data, reshaped into the exact {tracks, numTracks, egoPose} shape
%   generateFakeTracksAndPredictions.m produces - so buildRealOccupancyMap.m
%   and everything downstream works UNCHANGED against real M1 data.
%
%   matFilePath : path to m1_perception_day1.mat
%   timeQuery   : (optional) sim time in seconds to sample at. Defaults
%                 to the first timestep. Snaps to the nearest sample.
%
%   tracks    : 20x1 struct, SAME shape M1 already exports -
%               {id, class, x, y, heading, velocity, covariance, valid}.
%               class is a real AgentClass enum object.
%   numTracks : scalar - how many of tracks(1:numTracks) are valid.
%   egoPose   : [x, y, heading]. M1's field is 'yaw' - renamed here to
%               match 'heading', used consistently elsewhere in this
%               pipeline. Assumed WORLD frame (a vehicle's own pose can't
%               sensibly be expressed relative to itself) - meta.frame
%               only states perceptionData's frame (ego_ISO8855), NOT
%               egoData's. Worth a quick confirm with M1, not blocking.
%   timestamp : the REAL sim-time timestamp from perceptionData - NOT
%               navClock(). This is what M1/M4's Timestamp-field work
%               will eventually formalize on the bus.

S = load(matFilePath);

if ~isfield(S,'perceptionData') || ~isfield(S,'egoData') || ~isfield(S,'meta')
    error('loadM1RealPerception:BadFile', ...
        'Expected fields perceptionData/egoData/meta not found in %s.', matFilePath);
end

if nargin < 2 || isempty(timeQuery)
    idx = 1;
else
    [~, idx] = min(abs(S.perceptionData.time - timeQuery));
end

%% ---------- alignment check ----------
% perceptionData and egoData are two independently-timestamped signals.
% If they ever drift, indexing both by the same k silently misaligns
% perception and ego pose - check, don't assume.
if numel(S.perceptionData.time) ~= numel(S.egoData.time)
    error('loadM1RealPerception:LengthMismatch', ...
        'perceptionData has %d samples, egoData has %d - cannot index by a shared k.', ...
        numel(S.perceptionData.time), numel(S.egoData.time));
end
tDiff = abs(S.perceptionData.time(idx) - S.egoData.time(idx));
if tDiff > 1e-9
    warning('loadM1RealPerception:TimeMismatch', ...
        'perceptionData.time(%d)=%.4f but egoData.time(%d)=%.4f.', ...
        idx, S.perceptionData.time(idx), idx, S.egoData.time(idx));
end

%% ---------- extract ----------
rec = S.perceptionData.signals.values(idx);
tracks    = rec.tracks;        % already a 20x1 struct, per M1's contract
numTracks = rec.num_tracks;
timestamp = rec.timestamp;

egoRec  = S.egoData.signals.values(idx);
egoPose = [egoRec.x, egoRec.y, egoRec.yaw];
end