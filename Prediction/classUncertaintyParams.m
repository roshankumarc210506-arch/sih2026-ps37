function uncertaintyParams = classUncertaintyParams()
%CLASSUNCERTAINTYPARAMS Tunable per-class uncertainty growth parameters.
%
% Growth model:  r(t) = r0 + alpha * t^growthExponent
%
%   agentClass     : AgentClass enum value
%   r0             : base radius at t=0 (m)
%   alpha          : growth rate
%   growthExponent : 1 = linear, <1 = sub-linear/diffusion-like
%
% Fitted on Day 2 stub-sensor data (m1_perception_day1.mat) against
% measured prediction error, p95 per class, two-point fit at exponent=1.5.
% MUST be retuned once M1's Day 3 consolidated real-sensor export lands —
% sample sizes were thin for some classes at long horizons (see fit notes).

uncertaintyParams = struct( ...
'agentClass',     {AgentClass.Unknown, AgentClass.Car, AgentClass.TwoWheeler, AgentClass.AutoRickshaw, ...
                        AgentClass.PushCart, AgentClass.Pedestrian, AgentClass.Animal}, ...
'r0',             {2.351, 1.300, 1.186, 0.983, 0.770, 1.556, 0.983}, ...
'alpha',          {23.756, 10.159, 1.341, 3.363, 2.705, 2.814, 8.136}, ...
'growthExponent', {1.5,   1.5,    1.5,    1.5,    1.5,   1.5,   1.5});
end