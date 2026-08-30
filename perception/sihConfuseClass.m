function c = sihConfuseClass(trueClass, cfg)
%SIHCONFUSECLASS  Pick a plausible wrong label instead of a uniform-random one.
%
%   Consolidated here because this exact logic previously existed as a
%   private, duplicated function inside both sihDummyDetections.m and
%   sihRealDetections.m — same algorithm, two definitions, the same
%   pattern (shared logic silently duplicated across files) caught and
%   fixed four other times today for data/config, not code. LiDAR
%   clustering needs this too, making three call sites — worth one
%   shared definition rather than a third copy.

c = AgentClass.Unknown;
for i = 1:size(cfg.ConfusionPairs,1)
    if cfg.ConfusionPairs{i,1} == trueClass
        opts = cfg.ConfusionPairs{i,2};
        c    = opts(randi(numel(opts)));
        return
    end
end
end
