function [percOut, egoOut] = sihPerceptionPlaybackBlock()
%SIHPERCEPTIONPLAYBACKBLOCK  MATLAB Function block source. Plays back one
%frame of M1's real 248-frame perception data per simulation timestep,
%output typed as SihPerceptionBus / SihEgoBus.
%
% SETUP REQUIRED before this block will run:
%   1. In the model's InitFcn (or before opening the model), run:
%        load('data/m1_perception_day1.mat', 'perceptionData', 'egoData');
%        assignin('base', 'perceptionData', perceptionData);
%        assignin('base', 'egoData', egoData);
%   2. Add a MATLAB Function block to the Perception subsystem, paste
%      this function's body in, and set its two outputs' types to
%      Bus: SihPerceptionBus and Bus: SihEgoBus in Ports and Data
%      Manager (View > Ports and Data Manager in the block editor).
%
% WHY coder.extrinsic: sihGetPerceptionFrame does variable-size struct
% array indexing on data loaded from a .mat file -- not codegen-safe.
% coder.extrinsic marks it simulation-only: runs fine in Normal/Accelerator
% simulation mode, will NOT work if this model is ever used for code
% generation / Rapid Accelerator. Fine for Day 2 testing; flag before
% anyone tries to generate code from this subsystem later.

coder.extrinsic('sihGetPerceptionFrame');
coder.extrinsic('evalin');

persistent frameIdx
if isempty(frameIdx)
    frameIdx = 1;
end

percOut = Simulink.Bus.createMATLABStruct('SihPerceptionBus');
egoOut  = Simulink.Bus.createMATLABStruct('SihEgoBus');

% coder.extrinsic calls must assign into pre-typed outputs; MATLAB
% Function blocks require this two-step pattern (call, then let the
% (now-typed) result flow through) when the call itself isn't
% codegen-safe.
perceptionData = evalin('base', 'perceptionData'); %#ok<NASGU>
egoData        = evalin('base', 'egoData'); %#ok<NASGU>
numFrames      = numel(perceptionData.signals.values); %#ok<NODEF>

thisFrame = min(frameIdx, numFrames); % clamp: hold last frame after data runs out
[percOut, egoOut] = sihGetPerceptionFrame(perceptionData, egoData, thisFrame);

frameIdx = frameIdx + 1;
end