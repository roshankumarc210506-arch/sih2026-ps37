function open_scenario(sceneName)
%OPEN_SCENARIO Build and open one of the 5 required scenarios in
%   Driving Scenario Designer for visual refinement.
%
%   open_scenario('village')       -- unmarked village road
%   open_scenario('intersection')  -- unsignalled urban intersection
%   open_scenario('merge')         -- highway merge
%   open_scenario('market')        -- dense mixed-traffic market
%   open_scenario('cattle')        -- sudden cattle crossing

    switch lower(sceneName)
        case {'village', 'village_road', 'villageroad'}
            scenario = buildVillageRoadScenario();
        case {'intersection', 'urban_intersection', 'urbanintersection'}
            scenario = buildUrbanIntersectionScenario();
        case {'merge', 'highway_merge', 'highwaymerge'}
            scenario = buildHighwayMergeScenario();
        case {'market', 'dense_market', 'densemarket'}
            scenario = buildDenseMarketScenario();
        case {'cattle', 'cattle_crossing', 'cattlecrossing'}
            scenario = buildCattleCrossingScenario();
        otherwise
            error(['Unknown scenario "%s". Use one of: village, ' ...
                'intersection, merge, market, cattle.'], sceneName);
    end

    drivingScenarioDesigner(scenario)
end
