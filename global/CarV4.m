classdef CarV4 < SolarCar
    % CarV4 StarLight generation solar car.
    %   Subclass of SolarCar.m.
    %
    % See also SolarCar.
    properties

        % Car
        Name = 'StarLight'
        Wheelbase = 2550                % mm
        Trackwidth = 1270               % mm
        RawMass = 275                   % kg
        MinDriverMass = 80              % kg
        CoGx = 0.5                      % along wheelbase
        CoGy = 0.618                    % along trackwidth
        CoGh = 600                      % mm

        % Motor
        NominalMotorPower = 1470        % W
        PeakMotorPower = 5000           % W
        NominalSpeed = 40/3.6           % kph → m/s

        % Rack & Pinion (Stiletto C42-340 12:1)
        RPwidth = (15.225) * 25.4       % inch → mm             Kaz
        RPratio = (4.71) * 25.4         % inch/rev → mm/rev     6.4:1 - 4.712", 10:1 - 3.141", 12:1 - 2.618", 15:1 - 2.094", 20:1 - 1.570", Kaz - 4.71", NARRco - 3.46" & 4.0"
        RPmaxTravel = (4+5/8) * 25.4    % inch → mm

        % Wheel (Battlax SC 100/80-16 M/C 50P TL)
        WheelRadius = 566/2             % mm
        WheelCurvature = 54             % mm (radius)
        TirePressure = 65 / 145         % psi → N/mm²
        WheelStiffness = 115            % N/mm (50?)

        % Driving Surface
        RoadCrown = deg2rad(2)          % radians
    end
end
