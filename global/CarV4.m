classdef CarV4 < SolarCar
    % CarV4 StarLight generation solar car.
    %   Subclass of SolarCar.m.
    %
    % See also SolarCar.
    properties

        % Car
        Name = 'Cascadia'
        Wheelbase = 2600                % mm
        Trackwidth = 1270               % mm
        RawMass = 220                   % kg
        MinDriverMass = 80              % kg
        CoGx = 0.4694                   % along wheelbase
        CoGy = 0.6015                   % along trackwidth
        CoGh = 439.31                   % mm

        % Motor
        NominalMotorPower = 1470        % W
        PeakMotorPower = 5000           % W
        NominalSpeed = 40/3.6           % kph → m/s

        % Rack & Pinion (Kaz Tech)
        RPwidth = (15.225+2*0.8) * 25.4 % inch → mm
        Cfactor = (4.71) * 25.4         % inch/rev → mm/rev
        RPmaxTravel = 82.55    % inch → mm

        % Wheel (Battlax SC 100/80-16 M/C 50P TL)
        WheelRadius = 566/2             % mm
        WheelCurvature = 54             % mm (radius)
        TirePressure = 65 / 145         % psi → N/mm²
        WheelStiffness = 115            % N/mm (50?) doi:10.1088/1757-899X/776/1/012071

        % Driving Surface
        RoadCrown = deg2rad(2)          % radians
    end
end
