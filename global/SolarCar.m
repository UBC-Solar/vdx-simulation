classdef (Abstract) SolarCar
    % SolarCar Abstract base class for all solar vehicle generations.
    %   Defines shared properties and methods useful in modelling core vehicle dynamics.
    %
    %   Where possible, this class follows coordinate systems and naming conventions prescribed in ISO 8855.

    properties (Abstract)
        % Car
        Name
        Wheelbase           % mm
        Trackwidth          % mm
        RawMass             % kg
        MinDriverMass       % kg
        CoGx                % along wheelbase
        CoGy                % along trackwidth
        CoGh                % mm

        % Motor
        NominalMotorPower   % W
        PeakMotorPower      % W
        NominalSpeed        % m/s

        % Rack & Pinion
        RPwidth             % mm
        RPratio             % mm/rev
        RPmaxTravel         % mm

        % Wheel
        WheelRadius         % mm
        WheelCurvature      % mm (radius)
        TirePressure        % N/mm²
        WheelStiffness      % N/mm

        % Driving Surface
        % TODO: BECOME A NEW TRACK.M ABSTRACT CLASS
        RoadCrown           % radians
    end

    properties (Dependent)
        Mass                % kg
        Weight              % N
        MotorThrust         % N

        StaticLoadPB; StaticLoadPQ; StaticLoadSQ; StaticLoadSB % N
    end

    methods % for dependent properties
        function x = get.Mass(obj);         x = obj.RawMass + obj.MinDriverMass; end
        function x = get.Weight(obj);       x = obj.Mass * Phys.g; end
        function x = get.MotorThrust(obj);  x = obj.NominalMotorPower / obj.NominalSpeed; end

        function x = get.StaticLoadPB(obj); x = obj.solveLoad('PB'); end
        function x = get.StaticLoadPQ(obj); x = obj.solveLoad('PQ'); end
        function x = get.StaticLoadSQ(obj); x = obj.solveLoad('SQ'); end
        function x = get.StaticLoadSB(obj); x = obj.solveLoad('SB'); end
    end

    methods (Access = private)
        % TODO: MOVE TO AN ACCURATE DEFLECTION MODEL
        function load = solveLoad(obj, whlStr)                          % system of equations for static load distribution
            quadrantMap = struct('PB', 1, 'PQ', 2, 'SQ', 3, 'SB', 4);   % tire in quadrant n, following ISO 8855

            if ~isfield(quadrantMap, whlStr)
                error('invalid');
            end

            SYSTEM = @(V) [
                V(1) + V(2) + V(3) + V(4) - obj.Weight;         % force balance
                obj.CoGx*obj.Weight - V(1) - V(4);              % pitch moment balance
                V(1) + V(2) - obj.CoGy*obj.Weight;              % roll moment balance
                V(1) + V(3) - V(2) - V(4)                       % assume diagonals are equal
                ];  % the fourth equation solves the indeterminancy problem
            % only valid if Gx == 0.5 *or* Gy == 0.5 (otherwise function output is approximate)

            guessVec = 0.25*obj.Weight * ones(1, 4);
            solutionVec = fsolve(SYSTEM, guessVec, optimoptions('fsolve', 'Display', 'none'));
            load = solutionVec(quadrantMap.(whlStr));
        end
    end
end
