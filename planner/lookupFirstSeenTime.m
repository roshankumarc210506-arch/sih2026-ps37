function [t, found] = lookupFirstSeenTime(ids, firstSeenTimes, numIds, queryId)
%LOOKUPFIRSTSEENTIME  Linear-search replacement for containers.Map's
%   isKey/indexing, against the fixed-size parallel arrays
%   computeM1FirstSeenTimes.m now returns. containers.Map is not usable
%   inside a MATLAB Function block for Simulink - this is its
%   codegen-safe equivalent (fixed-size arrays + a simple loop, no
%   dynamic dictionary type).
%
%   ids, firstSeenTimes : Nx1 fixed-size arrays from computeM1FirstSeenTimes.m
%                         (only entries 1:numIds are valid - the rest is
%                         zero-padding, same convention as NumWaypoints/
%                         NumTracks elsewhere in this pipeline).
%   numIds              : how many leading entries of ids/firstSeenTimes
%                         are real.
%   queryId             : the track id to look up.
%
%   t     : the first-seen sim time, if found; 0 otherwise (caller must
%           check found, not just t - 0 is a valid real timestamp too).
%   found : true if queryId was present in ids(1:numIds).

t = 0;
found = false;
for j = 1:numIds
    if ids(j) == queryId
        t = firstSeenTimes(j);
        found = true;
        return;
    end
end
end