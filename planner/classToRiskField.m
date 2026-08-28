function riskField = classToRiskField(agentClass)
%CLASSTORISKFIELD  Map an AgentClass value onto a cfg.Risk.*_m field name.
%   NOTE: 'Pushcart' -> vehicle_m is an assumption, not yet confirmed with
%   the team. Flag before Day 3 handoff if pushcarts should get pedestrian-
%   level risk margin instead (they're slow/erratic like pedestrians, but
%   physically more like a small vehicle).

switch char(agentClass)
    case {'Car','TwoWheeler','AutoRickshaw','PushCart'}
        riskField = 'vehicle_m';
    case 'Pedestrian'
        riskField = 'pedestrian_m';
    case 'Animal'
        riskField = 'animal_m';
    case 'Unknown'
        riskField = 'unknown_m';
    otherwise
        error('classToRiskField:UnknownClass', ...
            'Unrecognized class "%s" - update this mapping.', char(agentClass));
end
end