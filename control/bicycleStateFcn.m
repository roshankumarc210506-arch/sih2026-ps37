function dxdt = bicycleStateFcn(x, u)
%BICYCLESTATEFCN Continuous-time kinematic bicycle model, rear-axle reference.
%
%   States  x = [X; Y; psi; v]
%       X, Y - global position of the REAR AXLE  [m]
%       psi  - heading / yaw                     [rad]
%       v    - longitudinal speed, SIGNED        [m/s]  (negative = reverse)
%
%   Inputs  u = [delta; a]
%       delta - front steering angle             [rad]
%       a     - longitudinal acceleration        [m/s^2]
%
%   Wheelbase is hardcoded (not read from vehicleParams) because nlmpc
%   generates code from this function and cannot call a params function
%   at build time. If vehicleParams.L changes, change it here too - the
%   assert in setup_m4_day1.m catches the mismatch.
%
%   dxdt is built by concatenation, not zeros(4,1), to stay compatible
%   with nlmpc's differentiation and codegen.

L = 2.5;    % wheelbase [m] - LOCKED, must match vehicleParams.L

dxdt = [ x(4) * cos(x(3));          % Xdot
         x(4) * sin(x(3));          % Ydot
         x(4) / L * tan(u(1));      % psidot
         u(2) ];                    % vdot
end
