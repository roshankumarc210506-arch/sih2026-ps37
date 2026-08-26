classdef DrivingMode < Simulink.IntEnumType
    % DRIVINGMODE Enum for the Stateflow decision-logic output.
    %
    % Matches the shared interface contract:
    %   Stateflow output -> driving_mode enum (CRUISE/CAUTIOUS/YIELD/STOP),
    %   broadcast as a bus signal; consumed by M4 ONLY (constraint
    %   tightening + velocity-profiler speed cap). Does NOT feed the
    %   occupancy map / inflation radii.
    %
    % This is a separate enum from M1's AgentClass.m (perception bus
    % agent classification). Don't conflate the two -- AgentClass
    % describes what an agent IS, DrivingMode describes what the EGO
    % should DO.
    enumeration
        CRUISE   (0)
        CAUTIOUS (1)
        YIELD    (2)
        STOP     (3)
    end

    methods (Static)
        function retVal = getDefaultValue()
            retVal = DrivingMode.CRUISE;
        end
    end
end
