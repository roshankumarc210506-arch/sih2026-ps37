function createPredictionBusObjects()
%CREATEPREDICTIONBUSOBJECTS Define only the PredictionBus (this module's
% output). Do NOT redefine TrackBus/PerceptionBus/EgoBus here - M6 has
% already published the shared definitions (SihTrackBus, SihPerceptionBus,
% SihEgoBus) to the repo. Load those directly so there is a single source
% of truth for the input side; only this module's output bus is defined
% below.
%
% Run this once (e.g. from a project startup script) before opening the
% Simulink model, after M6's shared bus script has already run.

N = 10; % must match the horizon N used when calling predictMotionCVCTR

% --- PredictionBus: one element of this module's output ---
predElems(1) = Simulink.BusElement; predElems(1).Name = 'id';                  predElems(1).DataType = 'uint32';
predElems(2) = Simulink.BusElement; predElems(2).Name = 'valid';               predElems(2).DataType = 'boolean';
predElems(3) = Simulink.BusElement; predElems(3).Name = 'predicted_positions'; predElems(3).DataType = 'double'; predElems(3).Dimensions = [N 2];
predElems(4) = Simulink.BusElement; predElems(4).Name = 'uncertainty_radius';  predElems(4).DataType = 'double'; predElems(4).Dimensions = [N 1];

SihPredictionBus = Simulink.Bus;
SihPredictionBus.Elements = predElems;
assignin('base', 'SihPredictionBus', SihPredictionBus);

fprintf('SihPredictionBus (N=%d) created in base workspace.\n', N);
