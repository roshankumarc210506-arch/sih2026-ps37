classdef AgentClass < Simulink.IntEnumType
    % AGENTCLASS  Agent classification for SIH 2026 PS37 perception output.
    %
    %   This is the `class` field of the shared perception contract:
    %       {id, class, x, y, heading, velocity, covariance}
    %
    %   IMPORTANT (raise with team on Day 1): a Simulink Bus cannot carry a
    %   MATLAB string/char field. `class` MUST be an enumeration (or uint8).
    %   Every module that reads the perception bus should use this enum,
    %   not a string literal. Cast with double(AgentClass.Car) -> 1.

    enumeration
        Unknown      (0)
        Car          (1)
        TwoWheeler   (2)
        AutoRickshaw (3)
        PushCart     (4)
        Pedestrian   (5)
        Animal       (6)
    end

    methods (Static)
        function defaultValue = getDefaultValue()
            defaultValue = AgentClass.Unknown;
        end

        function dScope = getDataScope()
            dScope = 'Exported';
        end

        function hdrFile = getHeaderFile()
            hdrFile = '';
        end

        function desc = getDescription()
            desc = 'SIH PS37 perception agent classes';
        end
    end
end
