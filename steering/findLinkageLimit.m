function maxYoke = findLinkageLimit(sNodes, ERaxisFun, findYokeAngle)
% findLinkageLimit - Find maximum yoke angle before tie rod overextension
%
%   maxYoke = findLinkageLimit(sNodes, ERaxisFun, findRackPos, findYokeAngle)
%
%   Uses port side geometry; result is symmetric for starboard.
%
%   Inputs:
%       sNodes          - Static node positions struct
%       ERaxisFun       - Function handle for extension rod position
%       findRackPos     - Function handle: yoke angle -> rack position
%       findYokeAngle   - Function handle: rack position -> yoke angle
%
%   Output:
%       maxYoke         - Maximum yoke angle [degrees] before overextension
%
%   See also: solveSteeringLinkage, steeringGeometry

arguments
    sNodes struct
    ERaxisFun function_handle
    findYokeAngle function_handle
end

% Calculate tie rod length and steering arm path
tieRodLen = norm(sNodes.ER_TR - sNodes.SA_TR);
KPaxisFun = @(t) sNodes.LBJ + t*((sNodes.UBJ - sNodes.LBJ) / norm(sNodes.UBJ - sNodes.LBJ));
SApathFun = getRevolvePath(KPaxisFun, sNodes.SA_TR);

% Function to find minimum distance from SA arc to ER connection point
minDistToER = @(rackShift) fminbnd(@(theta) norm(SApathFun(theta) - ERaxisFun(rackShift)), -pi, pi);

% Function to evaluate slack: negative when overextended
slackAtRack = @(rackShift) tieRodLen - norm(SApathFun(minDistToER(rackShift)) - ERaxisFun(rackShift));

% Find critical rack position where linkage just reaches limit (port side)
criticalRackPos = fzero(slackAtRack, [-200, 0]);

% Convert to yoke angle
maxYoke = findYokeAngle(criticalRackPos);
end
