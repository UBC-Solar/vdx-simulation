function [dNodes, spin, SApathFun] = solveSteeringLinkage(car, sNodes, ERaxisFun, rackShift, isPort)
% solveSteeringLinkage - Solve dynamic linkage positions for a given rack shift
%
%   [dNodes, spin, SApathFun] = solveSteeringLinkage(car, sNodes, ERaxisFun, rackShift, isPort)
%
%   Inputs:
%       car         - Car configuration object (e.g., CarV4)
%       sNodes      - Static node positions struct (TP, WC, UBJ, LBJ, KP, spindle, SA_TR, ER_TR, etc.)
%       ERaxisFun   - Function handle @(dy) returning extension rod connection point
%       rackShift   - Rack displacement [mm], positive = right turn
%       isPort      - +1 for port side, -1 for starboard side
%
%   Outputs:
%       dNodes      - Dynamic node positions after steering
%       spin        - Upright rotation angle [rad]
%       SApathFun   - Function handle for steering arm arc path
%
%   See also: steeringGeometry, getDelta, findLinkageLimit

arguments
    car
    sNodes struct
    ERaxisFun function_handle
    rackShift (1,1) double
    isPort (1,1) double {mustBeMember(isPort, [-1, 1])}
end

% Constraints for geometry solve
tieRodLen = norm(sNodes.ER_TR - sNodes.SA_TR);
KPaxisFun = @(t) sNodes.LBJ + t*((sNodes.UBJ - sNodes.LBJ) / norm(sNodes.UBJ - sNodes.LBJ));
SApathFun = getRevolvePath(KPaxisFun, sNodes.SA_TR);

dNodes.ER_TR = ERaxisFun(rackShift);
slack = @(theta) tieRodLen - norm(SApathFun(isPort*theta) - dNodes.ER_TR);  % key function

switch sign(rackShift) * isPort
    case 0  % static pos
        dNodes = sNodes;
        dNodes.TPrigid = sNodes.TP;
        spin = 0;
        return
    case 1  % steering arm swings CW (rack is approaching)
        thetaNegative = fminbnd(@(t) slack(t), -pi, 0);
        pathDomain = [thetaNegative 0];
    case -1  % steering arm swings CCW (rack is retreating)
        thetaPositive = fminbnd(@(t) -slack(t), 0, pi);  % closest pt on arc to ER_TR
        if slack(thetaPositive) < 0
            error("No upright orientation satisfies %f mm rack position!\nLinkage overextended!", rackShift);
        end
        pathDomain = [0 thetaPositive];
end

% Solve for upright rotation angle
spin = isPort * fzero(slack, pathDomain);  % [rad]

% Compute dynamic node positions
dNodes.SA_TR = SApathFun(spin);
dNodes.TPrigid = rotAboutAxis(KPaxisFun, sNodes.TP, spin);
dNodes.WC = rotAboutAxis(KPaxisFun, sNodes.WC, spin);
dNodes.spindle = rotAboutAxis(KPaxisFun, sNodes.spindle, spin);

% Static nodes unchanged
dNodes.UBJ = sNodes.UBJ;
dNodes.LBJ = sNodes.LBJ;
dNodes.KP = sNodes.KP;
dNodes.KP_SA = sNodes.KP_SA;

% Find tire contact patch (lowest point on wheel)
dNodes.TP = ringPointMinZ(dNodes.WC, dNodes.WC - dNodes.spindle, car.WheelRadius);
end
