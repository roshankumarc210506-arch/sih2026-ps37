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
% Placeholder values for the Day-1 stub - retune against real scenario
% logs once available.

uncertaintyParams = struct( ...
    'agentClass',     {AgentClass.Unknown, AgentClass.Car, AgentClass.TwoWheeler, AgentClass.AutoRickshaw, ...
                        AgentClass.PushCart, AgentClass.Pedestrian, AgentClass.Animal}, ...
    'r0',             {0.40, 0.30, 0.30, 0.35, 0.35, 0.40, 0.40}, ...
    'alpha',          {0.70, 0.15, 0.35, 0.30, 0.30, 0.70, 0.90}, ...
    'growthExponent', {0.7,  1,    1,    1,    1,    0.7,  0.6});

end