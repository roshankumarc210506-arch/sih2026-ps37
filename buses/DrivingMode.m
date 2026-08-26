classdef DrivingMode < Simulink.IntEnumType
%DRIVINGMODE Decision-logic mode enum, produced by M5's Stateflow chart.
%
%   Defined by M4 because M4 is the sole consumer (per locked contract).
%   M5: reference these symbols in your chart output, e.g.
%       driving_mode = DrivingMode.CAUTIOUS;
%
%   Same reason as M1's AgentClass.m - Simulink buses cannot carry strings,
%   so this must be an enum, not 'cautious'.
%
%   Numeric values are ordered by severity (CRUISE < CAUTIOUS < YIELD < STOP)
%   so that if two sources ever disagree we can take max() and fail safe.

    enumeration
        CRUISE   (0)
        CAUTIOUS (1)
        YIELD    (2)
        STOP     (3)
    end

    methods (Static)
        function retVal = getDefaultValue()
            % FAIL-SAFE default. If the bus is uninitialised, if Stateflow
            % has not run yet, or if the signal drops, we stop - we do not
            % cruise. Do not change this to CRUISE for convenience.
            retVal = DrivingMode.STOP;
        end

        function retVal = getDescription()
            retVal = 'Driving mode from Stateflow decision logic (M5 -> M4)';
        end

        function retVal = addClassNameToEnumNames()
            retVal = true;
        end
    end
end
