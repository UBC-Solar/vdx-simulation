function df = genPerformanceTable(car, sNodes, sNodesStarboard, ERaxisFun, ERaxisFunStarboard, findRackPos, maxYoke, n, twoSided)
% genPerformanceTable - Generate steering performance analysis data
%
%   df = genPerformanceTable(car, sNodes, sNodesStarboard, ERaxisFun, ...
%           ERaxisFunStarboard, findRackPos, maxYoke, n, twoSided)
%
%   Sweeps through yoke positions and computes all steering performance metrics.
%
%   Inputs:
%       car                 - Car configuration object (e.g., CarV4)
%       sNodes              - Static node positions (port side)
%       sNodesStarboard     - Static node positions (starboard side)
%       ERaxisFun           - Extension rod axis function (port)
%       ERaxisFunStarboard  - Extension rod axis function (starboard)
%       findRackPos         - Function: yoke angle -> rack position
%       maxYoke             - Maximum yoke angle [degrees]
%       n                   - Number of sweep points (default: 128)
%       twoSided            - If true, sweep from -maxYoke to +maxYoke (default: true)
%
%   Output:
%       df                  - Table with all steering performance channels
%
%   See also: steeringGeometry, solveSteeringLinkage, steeringPerformance

arguments
    car
    sNodes struct
    sNodesStarboard struct
    ERaxisFun function_handle
    ERaxisFunStarboard function_handle
    findRackPos function_handle
    maxYoke (1,1) double
    n (1,1) double = 128
    twoSided (1,1) logical = true
end

wheelbase = car.Wheelbase;
trackwidth = car.Trackwidth;

% Initialize Sweep Parameters
df = table();
maxYokeNeg = -maxYoke;

df.yokePos = linspace(twoSided * maxYokeNeg, maxYoke, n)';
df.rackPos = -findRackPos(df.yokePos);  % TODO: Interim solution, verify sign convention

% Solve Port Side Nodes
df.spinPort = zeros(n, 1);
for i = 1:n
    [nodesPortStructArray(i), df.spinPort(i), ~] = ...
        solveSteeringLinkage(car, sNodes, ERaxisFun, df.rackPos(i), 1);
end

% Solve Starboard Side Nodes
df.spinStarboard = zeros(n, 1);
for i = 1:n
    [nodesStarboardStructArray(i), df.spinStarboard(i), ~] = ...
        solveSteeringLinkage(car, sNodesStarboard, ERaxisFunStarboard, df.rackPos(i), -1);
end

% Calculate Steer Angles
df.deltaL = rad2deg(arrayfun(@(s) getDelta(s.WC, s.spindle), nodesPortStructArray))';
df.deltaR = rad2deg(arrayfun(@(s) getDelta(s.WC, s.spindle), nodesStarboardStructArray))';
df.deltaEff = acotd((cotd(df.deltaL) + cotd(df.deltaR)) ./ 2);

% Determine Outer/Inner Wheel Based on Turn Direction
df.deltaOuter = zeros(n, 1);
df.deltaInner = zeros(n, 1);
for i = 1:n
    if df.rackPos(i) > 0  % right turn
        df.deltaOuter(i) = df.deltaL(i);
        df.deltaInner(i) = df.deltaR(i);
    else  % left turn
        df.deltaOuter(i) = df.deltaR(i);
        df.deltaInner(i) = df.deltaL(i);
    end
end

% Calculate Ackermann Geometry Metrics
df.deltaInnerIdeal = sign(df.deltaOuter) .* acotd(cotd(abs(df.deltaOuter)) - trackwidth/wheelbase);
df.innerError = (df.deltaInner - df.deltaInnerIdeal) ./ df.deltaInnerIdeal;
df.dynamicToe = abs(df.deltaOuter) - abs(df.deltaInner);
df.dynamicToeIdeal = abs(df.deltaOuter) - abs(df.deltaInnerIdeal);

% Calculate Instantaneous Centers and Turn Radii
df.intL = zeros(n, 1);
df.intR = zeros(n, 1);

for i = 1:n
    if abs(df.deltaL(i)) > eps
        df.intL(i) = getICintercept(nodesPortStructArray(i).TP, deg2rad(df.deltaL(i)));
    else
        df.intL(i) = inf;
    end
    if abs(df.deltaR(i)) > eps
        df.intR(i) = getICintercept(nodesStarboardStructArray(i).TP, deg2rad(df.deltaR(i)));
    else
        df.intR(i) = inf;
    end
end

% Calculate Combined Turning Metrics
df.radiusEff = abs(wheelbase ./ sind(df.deltaEff));
df.turnCentre = sign(df.deltaEff) .* sqrt(df.radiusEff.^2 - wheelbase.^2);

% Calculate Inner and Outer Radii from Effective Radius
df.radiusInner = zeros(n, 1);
df.radiusOuter = zeros(n, 1);
for i = 1:n
    if abs(df.deltaEff(i)) > eps
        % Distance from turn center to inner and outer wheels
        df.radiusInner(i) = norm([wheelbase, abs(df.turnCentre(i)) - trackwidth/2]);
        df.radiusOuter(i) = norm([wheelbase, abs(df.turnCentre(i)) + trackwidth/2]);
    else
        df.radiusInner(i) = inf;
        df.radiusOuter(i) = inf;
    end
end
df.ICdist = -(df.intL - df.intR);  % inner to outer
df.speedForLateral1G = sqrt(Phys.g * abs(df.radiusEff/1e3));

% Calculate Parasitic Camber
df.parasiticCamberL = zeros(n, 1);
df.parasiticCamberR = zeros(n, 1);
for i = 1:n
    hVecL = nodesPortStructArray(i).TP(1:2) - nodesPortStructArray(i).WC(1:2);
    hVecR = nodesStarboardStructArray(i).TP(1:2) - nodesStarboardStructArray(i).WC(1:2);

    distOverL = sign(hVecL(1)) * norm(hVecL);
    distOverR = sign(hVecR(1)) * norm(hVecR);

    heightL = nodesPortStructArray(i).WC(3) - nodesPortStructArray(i).TP(3);
    heightR = nodesStarboardStructArray(i).WC(3) - nodesStarboardStructArray(i).TP(3);

    df.parasiticCamberL(i) = atand(distOverL/heightL);
    df.parasiticCamberR(i) = atand(distOverR/heightR);
end

% Calculate Motion Angles
df.betaMotionL = zeros(n, 1);
df.betaMotionR = zeros(n, 1);
for i = 1:n
    df.betaMotionL(i) = atand(nodesPortStructArray(i).TP(1) ./ (df.turnCentre(i) - nodesPortStructArray(i).TP(2)));
    df.betaMotionR(i) = atand(nodesStarboardStructArray(i).TP(1) ./ (df.turnCentre(i) - nodesStarboardStructArray(i).TP(2)));
end

df.inducedAlphaL = df.deltaL - df.betaMotionL;
df.inducedAlphaR = df.deltaR - df.betaMotionR;

% Calculate Induced Power Losses
df.inducedPowerLossL = arrayfun(@(a) slip2latForce(deg2rad(abs(a)), 0.25*car.Weight, car), df.inducedAlphaL);
df.inducedPowerLossR = arrayfun(@(a) slip2latForce(deg2rad(abs(a)), 0.25*car.Weight, car), df.inducedAlphaR);
df.inducedPowerLossTot = df.inducedPowerLossL + df.inducedPowerLossR;
end
