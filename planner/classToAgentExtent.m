function extent_m = classToAgentExtent(agentClass)
%CLASSTOAGENTEXTENT  Per-class worst-case diagonal extent (m), used as a
%   conservative radius added around an agent's reported near-surface
%   point (M1 confirmed: reported (x,y) is near-surface, not centroid -
%   Session Context 3i).
%
%   Using the FULL diagonal (not half) as a radius is the "diagonal /
%   full-circle inflation" stopgap M1 explicitly confirmed sound: since
%   the true object only extends AWAY from ego from the reported point,
%   the full diagonal conservatively covers the far side of the object
%   as well, without needing to know which way it's oriented.
%
%   STOPGAP, not final. M1 declined to provide a real per-class offset
%   table (one sample != a stable class property - "fake precision").
%   Values below are M1's own scenario-construction dimensions (exact,
%   not estimated), current as of the PushCart 1.6m sync fix (3i.7).
%   Superseded once LiDAR clustering lands and gives a real observed
%   bounding box per object (3i.6).
%
%   Do NOT swap this for M1's oriented-rectangle proposal (3i.4) without
%   a contract change / team notify - see Open Items #8. This function
%   is the isotropic stopgap only.
%
%   agentClass may be an AgentClass enum object or a char/string; always
%   coerced via char() since real perception data (m1_perception_day1.mat)
%   confirmed 'class' is a genuine MCOS enum, not a string (3g).

switch char(agentClass)
    case 'Car'
        extent_m = 4.85;
    case 'TwoWheeler'
        extent_m = 2.03;
    case 'AutoRickshaw'
        extent_m = 2.95;
    case 'PushCart'
        extent_m = 1.83;
    case 'Pedestrian'
        extent_m = 0.78;
    case 'Animal'
        extent_m = 2.38;
    case 'Unknown'
        extent_m = 4.85;
    otherwise
        error('classToAgentExtent:UnknownClass', ...
            'No extent defined for agent class ''%s''.', char(agentClass));
end
end