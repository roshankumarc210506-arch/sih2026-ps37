function [agent_density, risk_zone_high_risk_agent] = computeDecisionSignals(tracks, num_tracks)
%COMPUTEDECISIONSIGNALS Agent density + high-risk-zone flag from M1's track list.
%
%   [agent_density, risk_zone_high_risk_agent] = computeDecisionSignals(tracks, num_tracks)
%
%   Inputs (per the shared perception interface contract):
%     tracks     - struct array, fixed length 20. Each element:
%                  .id, .class (AgentClass enum), .x, .y, .heading,
%                  .velocity, .covariance, .valid
%     num_tracks - number of VALID entries at the front of tracks.
%                  Loop 1:num_tracks, not 1:numel(tracks).
%
%   Outputs:
%     agent_density              - count of valid tracks within
%                                   DENSITY_ZONE_RADIUS_M of ego (ego at
%                                   origin, ego-vehicle ISO 8855 frame).
%     risk_zone_high_risk_agent  - true if any valid Pedestrian or
%                                   Animal track is within
%                                   RISK_ZONE_RADIUS_M of ego.
%
%   Both radii are PLACEHOLDERS -- retune once this runs against real
%   scenario data (Phase 1, Days 2-4), same status as the chart's
%   density thresholds.

DENSITY_ZONE_RADIUS_M = 30;   % PLACEHOLDER
RISK_ZONE_RADIUS_M = 15;      % PLACEHOLDER

agent_density = 0;
risk_zone_high_risk_agent = false;

for i = 1:num_tracks
    t = tracks(i);
    if ~t.valid
        continue;
    end

    dist = hypot(t.x, t.y);

    if dist <= DENSITY_ZONE_RADIUS_M
        agent_density = agent_density + 1;
    end

    if dist <= RISK_ZONE_RADIUS_M && ...
            (t.class == AgentClass.Pedestrian || t.class == AgentClass.Animal)
        risk_zone_high_risk_agent = true;
    end
end
end