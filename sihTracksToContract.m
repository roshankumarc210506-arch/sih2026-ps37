function [trackArray, numTracks] = sihTracksToContract(tracks, classes, cfg)
%SIHTRACKSTOCONTRACT  Tracker output -> the locked interface contract.
%
%   [trackArray, n] = sihTracksToContract(tracks, classes, cfg)
%
%   THE ONE INTERESTING BIT — the covariance transform.
%
%   The tracker's constant-velocity EKF carries state
%       s = [x, vx, y, vy, z, vz]         with 6x6 covariance P.
%   The contract exposes
%       [x, y, heading, velocity]         with 4x4 covariance.
%
%   heading and velocity are NONLINEAR functions of the state:
%       theta = atan2(vy, vx)
%       v     = sqrt(vx^2 + vy^2)
%
%   So you cannot just slice P. You must push it through the Jacobian:
%       P_contract = J * P * J'      where J = d[x,y,theta,v]/ds   (4x6)
%
%   Slicing P instead of transforming it is the single most likely silent
%   bug in this module: M2 would size his uncertainty cones off a heading
%   variance that is simply wrong, and nothing would visibly break.
%
%   Near-stationary guard: as v -> 0, dtheta/dv blows up. Below
%   cfg.MinSpeedForHeading the heading is meaningless, so we clamp the
%   Jacobian and inject a large fixed heading variance instead. This
%   matters a lot here — pushcarts, parked autos and grazing cattle spend
%   most of the scenario below that threshold.

if nargin < 3, cfg = sihConfig(); end

trackArray = repmat(sihEmptyTrack(), cfg.MaxTracks, 1);
numTracks  = uint32(min(numel(tracks), cfg.MaxTracks));

for i = 1:double(numTracks)
    s = tracks(i).State;              % [x vx y vy z vz]
    P = tracks(i).StateCovariance;    % 6x6

    x  = s(1);  vx = s(2);
    y  = s(3);  vy = s(4);

    v     = hypot(vx, vy);
    theta = atan2(vy, vx);

    % ---- Jacobian J = d[x, y, theta, v] / d[x vx y vy z vz] ----
    J = zeros(4, numel(s));
    J(1,1) = 1;                       % dx/dx
    J(2,3) = 1;                       % dy/dy

    slow = v < cfg.MinSpeedForHeading;
    if slow
        % Heading is not observable; leave its Jacobian row at zero and add
        % the variance back explicitly below.
        J(4,2) = 0;  J(4,4) = 0;
    else
        n2     = vx^2 + vy^2;
        J(3,2) = -vy / n2;            % dtheta/dvx
        J(3,4) =  vx / n2;            % dtheta/dvy
        J(4,2) =  vx / v;             % dv/dvx
        J(4,4) =  vy / v;             % dv/dvy
    end

    C = J * P * J';

    if slow
        C(3,3) = cfg.StationaryHeadingVar;
        % Speed variance still comes from the velocity block of P.
        C(4,4) = max(C(4,4), P(2,2) + P(4,4));
    end

    C = 0.5 * (C + C');                       % force symmetry
    C = C + 1e-9 * eye(4);                    % keep it positive definite

    trackArray(i).id         = uint32(tracks(i).TrackID);
    trackArray(i).class      = classes(i);
    trackArray(i).x          = x;
    trackArray(i).y          = y;
    trackArray(i).heading    = sihWrapToPi(theta);
    trackArray(i).velocity   = v;
    trackArray(i).covariance = C;
    trackArray(i).valid      = true;
end
end

% ------------------------------------------------------------------------
function t = sihEmptyTrack()
t = struct( ...
    'id',         uint32(0), ...
    'class',      AgentClass.Unknown, ...
    'x',          0, ...
    'y',          0, ...
    'heading',    0, ...
    'velocity',   0, ...
    'covariance', zeros(4,4), ...
    'valid',      false);
end

% ------------------------------------------------------------------------
function a = sihWrapToPi(a)
a = mod(a + pi, 2*pi) - pi;
end
