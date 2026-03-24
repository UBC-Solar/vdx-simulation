clear; format shortG; close all;

%% Load Steering Geometry
SAvec = [-92, 18];
ERconnectionLen = 50;
setback = 390;
steeringGeometry;

numPoints = 511;
twoSided = true;
df = genPerformanceTable(car, sNodes, sNodesStarboard, ERaxisFun, ERaxisFunStarboard, findRackPos, maxYoke, numPoints, twoSided);

%% Physical Constraints
maxPneumaticTrail = car.WheelRadius * 0.05;         % Approximate max pneumatic trail offset [mm]
pneumaticTrailSensitivity = 0.012;                  % Estimated trail decay [mm/N]
pinion_radius = car.Cfactor / (2 * pi);
mu_rack = 0.15;                                     % Linear bushing friction coefficient
frontAxleMass = (car.StaticLoadPB + car.StaticLoadSB) / Phys.g;

%% Calculate Steering Effort over the Geometry Matrix
fprintf('\nPropagating forces through dynamic linkage...\n');

kph = [5, 10, 15, 20, 30, 40, 55, 80];
velocities = kph ./ 3.6;
numVelCurves = length(velocities);

lateralAccelMatrix   = zeros(numPoints, numVelCurves);
torqueMatrixRaw      = zeros(numPoints, numVelCurves);
torqueMatrixFeedback = zeros(numPoints, numVelCurves);
torqueMatrixActive   = zeros(numPoints, numVelCurves);
bushingNxMatrix      = zeros(numPoints, numVelCurves);
bushingNzMatrix      = zeros(numPoints, numVelCurves);

for i = 1:numPoints

    [dNodesPort, ~, ~] = solveSteeringLinkage(car, sNodes, ERaxisFun, df.rackPos(i), 1);
    [dNodesStbd, ~, ~] = solveSteeringLinkage(car, sNodesStarboard, ERaxisFunStarboard, df.rackPos(i), -1);

    deltaL = deg2rad(df.deltaL(i));
    deltaR = deg2rad(df.deltaR(i));

    R = df.radiusEff(i) / 1000;  % Kinematic turning radius [m]

    % Static kingpin axis
    KP_axis_L = dNodesPort.UBJ - dNodesPort.LBJ;
    KP_axis_L = KP_axis_L / norm(KP_axis_L);
    KP_axis_R = dNodesStbd.UBJ - dNodesStbd.LBJ;
    KP_axis_R = KP_axis_R / norm(KP_axis_R);

    % Local Tire Coordinate System (ISO 8855)
    wheelHeadL = [cos(deltaL); sin(deltaL); 0];
    wheelHeadR = [cos(deltaR); sin(deltaR); 0];
    wheelLatL = [-sin(deltaL); cos(deltaL); 0];
    wheelLatR = [-sin(deltaR); cos(deltaR); 0];

    % Inner loop for handling variations across velocity thresholds
    for j = 1:numVelCurves
        v = velocities(j);

        % Dynamic lateral acceleration for this curve based strictly on kinematics
        if isinf(R) || df.rackPos(i) == 0
            a_y = 0;
        else
            % rackPos > 0 = right turn (-a_y)
            a_y = -(v^2 / R) * sign(df.rackPos(i));
        end
        lateralAccelMatrix(i, j) = a_y;

        % Weight transfer
        Fz_L = max(0, car.StaticLoadPB - frontAxleMass * a_y * (car.CoGh / car.Trackwidth));
        Fz_R = max(0, car.StaticLoadSB + frontAxleMass * a_y * (car.CoGh / car.Trackwidth));

        % Proportion lateral force
        Fy_L = (frontAxleMass * a_y) * (Fz_L / (Fz_L + Fz_R));
        Fy_R = (frontAxleMass * a_y) * (Fz_R / (Fz_L + Fz_R));

        % Locate Center of Pressure (Pneumatic Trail rollback)
        % Decreases linearly with applied lateral force
        t_p_L = max(0, maxPneumaticTrail - pneumaticTrailSensitivity * abs(Fy_L));
        t_p_R = max(0, maxPneumaticTrail - pneumaticTrailSensitivity * abs(Fy_R));

        % 3D pos force acts on tire patch
        CoP_L = dNodesPort.TP - wheelHeadL * t_p_L;
        CoP_R = dNodesStbd.TP - wheelHeadR * t_p_R;

        % Combined tire force vector
        Fvec_L = Fy_L * wheelLatL + Fz_L * [0; 0; 1];
        Fvec_R = Fy_R * wheelLatR + Fz_R * [0; 0; 1];

        % Moments about kingpins
        % Cross Product finds the total 3D torque around the pivot point
        % Dot Product projects that torque onto the kingpin axis
        % Note: r vector can originate at any point along KP axis, gets cancelled out
        Mkp_L = dot(cross(CoP_L - dNodesPort.LBJ, Fvec_L), KP_axis_L);
        Mkp_R = dot(cross(CoP_R - dNodesStbd.LBJ, Fvec_R), KP_axis_R);

        % Resulting axial force in tie rods (counter tire moment)
        TRvec_L = dNodesPort.ER_TR - dNodesPort.SA_TR;
        TRvec_L = TRvec_L / norm(TRvec_L);
        TRvec_R = dNodesStbd.ER_TR - dNodesStbd.SA_TR;
        TRvec_R = TRvec_R / norm(TRvec_R);

        % Instantaneous leverage (effective moment arm) of tie rod around kingpin [mm]
        tauL = dot(cross(dNodesPort.SA_TR - dNodesPort.UBJ, TRvec_L), KP_axis_L);
        tauR = dot(cross(dNodesStbd.SA_TR - dNodesStbd.UBJ, TRvec_R), KP_axis_R);

        % Axial forces in tie rods (scalar magnitude)
        Ftr_L = -Mkp_L / tauL;  % [N]
        Ftr_R = -Mkp_R / tauR;  % [N]

        % Resulting axial forces in the extension rod/rack
        Frack_L = -Ftr_L * TRvec_L(2);
        Frack_R = -Ftr_R * TRvec_R(2);

        % Normal forces acting on the linear bushings (X and Z components of tie rod forces)
        % Positive = bushing resists upward (Z) or forward (X) motion of rack
        Nx_rack = Ftr_L * TRvec_L(1) + Ftr_R * TRvec_R(1);  % Longitudinal [N]
        Nz_rack = Ftr_L * TRvec_L(3) + Ftr_R * TRvec_R(3);  % Vertical [N]
        Nrack = abs(Nx_rack) + abs(Nz_rack);
        % Note: neglects normal force to oppose couple moment of the two tie rod forces

        % Friction opposing the force transmitted from tires to the driver
        Ffric = Nrack * mu_rack;

        % Net force from tires pushing on the rack
        Frack_net = Frack_L + Frack_R;

        % 1) Raw: tire moment transmitted directly, no friction
        Tyoke_raw = -Frack_net * pinion_radius;

        % 2) Feedback: bushing friction absorbs some back-drive before it reaches the driver
        if abs(Frack_net) <= Ffric
            Frack_transmitted = 0;
        else
            Frack_transmitted = Frack_net - sign(Frack_net) * Ffric;
        end
        Tyoke_feedback = -Frack_transmitted * pinion_radius;

        % 3) Active steer: driver steers against tire moments; friction now adds to resistance
        Tyoke_active = (Frack_net + sign(-df.rackPos(i)) * Ffric) * pinion_radius;

        torqueMatrixRaw(i, j)      = Tyoke_raw      / 1000; % N-mm to Nm
        torqueMatrixFeedback(i, j) = Tyoke_feedback / 1000;
        torqueMatrixActive(i, j)   = Tyoke_active   / 1000;
        bushingNxMatrix(i, j)      = Nx_rack;
        bushingNzMatrix(i, j)      = Nz_rack;
    end
end

%% Generate Plots

plotColors = flipud(nebula(numVelCurves));
centerIdx  = find(abs(df.rackPos) < 1e-4, 1);

plotSteeringEffort(torqueMatrixRaw,      lateralAccelMatrix, kph, plotColors, centerIdx, 'Feedback Torque, Idealized Force Transmission');
plotSteeringEffort(torqueMatrixFeedback, lateralAccelMatrix, kph, plotColors, centerIdx, 'Feedback Torque, Considering Bushing Friction');
plotSteeringEffort(torqueMatrixActive,   lateralAccelMatrix, kph, plotColors, centerIdx, 'Driver Input Torque, Considering Bushing Friction');

%% Bushing Normal Forces
radiusMeters = df.radiusEff / 1000;  % Convert mm to m
[~, idxR] = min(abs(radiusMeters - 7.5));  % Find closest point to 7.5 m
actualRadius = radiusMeters(idxR);

figure(Name='Bulkhead Forces'); hold on;
title(sprintf('Extension Rod Bushing Forces (Combined) @ R = %.2f m', actualRadius));
xlabel('Speed [km/h]');
ylabel('Total Bulkhead Force [N]');
grid on;

plot(kph, bushingNxMatrix(idxR, :), '_r', MarkerSize=16, LineWidth=4, DisplayName='Longitudinal (X)');
plot(kph, bushingNzMatrix(idxR, :), '_b', MarkerSize=16, LineWidth=4, DisplayName='Vertical (Z)');
legend(Location='northeast');

disp("Complete!");
