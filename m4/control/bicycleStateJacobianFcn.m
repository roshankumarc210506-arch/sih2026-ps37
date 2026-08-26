function [A, B] = bicycleStateJacobianFcn(x, u)
%BICYCLESTATEJACOBIANFCN Analytic Jacobians of bicycleStateFcn.
%
%   A = d(dxdt)/dx  (4x4),  B = d(dxdt)/du  (4x2)
%
%   Supplying this to nlmpc replaces finite-difference Jacobian estimation
%   and typically cuts solve time 2-5x. Since M6 is logging control-loop
%   timing as a reported metric, wire this in from Day 1, not later.

L = 2.5;    % MUST match bicycleStateFcn

psi   = x(3);
v     = x(4);
delta = u(1);

A = [ 0, 0, -v*sin(psi),  cos(psi);
      0, 0,  v*cos(psi),  sin(psi);
      0, 0,           0,  tan(delta)/L;
      0, 0,           0,  0 ];

B = [ 0,                   0;
      0,                   0;
      v/(L*cos(delta)^2),  0;
      0,                   1 ];
end
