classdef carV4
    properties (Constant)
        % see ISO 8855 for coordinate systems etc.

        % Constants
        g = 9.80665                     % m/s²

        % Car
        Wheelbase = 2550                % mm
        Trackwidth = 1270               % mm
        RawMass = 220                   % kg
        MinDriverMass = 80              % kg
        CoGx = 0.55                     % along wheelbase
        CoGy = 0.5                      % along trackwidth
        CoGh = 600                      % mm

        % Motor
        NominalMotorPower = 1470        % W
        PeakMotorPower = 5000           % W
        NominalSpeed = 40/3.6           % kph → m/s

        % Rack & Pinion (Stiletto C42-340 12:1)
        RPwidth = (11.25) * 25.4        % inch → mm
        RPratio = (12) * 25.4           % inch/rev → mm/rev
        RPmaxTravel = (4+5/8) * 25.4    % inch → mm

        % Wheel (Battlax SC 100/80-16 M/C 50P TL)
        WheelRadius = 566/2             % mm
        WheelCurvature = 54             % mm (radius)
        TirePressure = 65 / 145         % psi → N/mm²
        WheelStiffness = 115             % N/mm

        % Driving Surface
        RoadCrown = deg2rad(2)          % radians
    end

    properties (Dependent)
        Mass                            % kg
        Weight                          % N
        MotorThrust                     % N

        StaticLoadPB                    % N
        StaticLoadPQ                    % N
        StaticLoadSQ                    % N
        StaticLoadSB                    % N
    end

    methods % for dependent properties
        function x = get.Mass(obj);         x = obj.RawMass + obj.MinDriverMass; end
        function x = get.Weight(obj);       x = obj.Mass * obj.g; end
        function x = get.MotorThrust(obj);  x = obj.NominalMotorPower / obj.NominalSpeed; end

        function x = get.StaticLoadPB(obj); x = obj.solveLoad('PB'); end
        function x = get.StaticLoadPQ(obj); x = obj.solveLoad('PQ'); end
        function x = get.StaticLoadSQ(obj); x = obj.solveLoad('SQ'); end
        function x = get.StaticLoadSB(obj); x = obj.solveLoad('SB'); end
    end

    methods (Access = private)
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
