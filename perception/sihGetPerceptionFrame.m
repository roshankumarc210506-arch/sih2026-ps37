function [percOut, egoOut] = sihGetPerceptionFrame(perceptionData, egoData, frameIdx)
%SIHGETPERCEPTIONFRAME  Extract and reshape one frame of M1's real
%perception data into structs matching SihPerceptionBus / SihEgoBus
%exactly, for playback via a MATLAB Function block (From Workspace
%rejects this data directly since it's a nested bus-shaped struct, not
%a flat numeric/logical/enum signal).
%
% Called with SIMULATION-ONLY semantics (via coder.extrinsic in the
% wrapper block) since it does variable-size/struct indexing that isn't
% codegen-friendly -- fine for Day 2 playback/testing, NOT meant to be
% part of a deployable/codegen path.
%
%   frameIdx : 1-based frame index into perceptionData.signals.values /
%              egoData.signals.values. Caller is responsible for
%              clamping to valid range (see wrapper block).

    pFrame = perceptionData.signals.values(frameIdx);
    eFrame = egoData.signals.values(frameIdx);

    % ---- Perception output: must match SihPerceptionBus exactly ----
    % (tracks[40] of SihTrackBus, num_tracks uint32, timestamp double)
    percOut = Simulink.Bus.createMATLABStruct('SihPerceptionBus');

    numTracks = min(pFrame.num_tracks, numel(percOut.tracks));
    for i = 1:numTracks
        percOut.tracks(i).id         = uint32(pFrame.tracks(i).id);
        percOut.tracks(i).class      = pFrame.tracks(i).class;   % must already be AgentClass enum in the .mat
        percOut.tracks(i).x          = double(pFrame.tracks(i).x);
        percOut.tracks(i).y          = double(pFrame.tracks(i).y);
        percOut.tracks(i).heading    = double(pFrame.tracks(i).heading);
        percOut.tracks(i).velocity   = double(pFrame.tracks(i).velocity);
        percOut.tracks(i).covariance = double(pFrame.tracks(i).covariance);
        percOut.tracks(i).valid      = true;
    end
    % remaining slots stay at createMATLABStruct's zero-init default,
    % with valid=false -- consumers must loop 1:num_tracks, per contract
    percOut.num_tracks = uint32(numTracks);
    percOut.timestamp  = double(pFrame.timestamp);

    % ---- Ego output: must match SihEgoBus exactly (5 fields) ----
    egoOut = Simulink.Bus.createMATLABStruct('SihEgoBus');
    egoOut.x        = double(eFrame.x);
    egoOut.y        = double(eFrame.y);
    egoOut.yaw      = double(eFrame.yaw);
    egoOut.velocity = double(eFrame.velocity);
    if isfield(eFrame, 'Timestamp')
        egoOut.Timestamp = double(eFrame.Timestamp);
    else
        egoOut.Timestamp = double(pFrame.timestamp); % fallback: reuse perception frame's timestamp
    end
end