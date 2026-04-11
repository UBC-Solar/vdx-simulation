%% Steering Geometry Configuration
%  This script initializes all steering linkage geometry parameters into workspace.
%  steeringVis.m or steeringPerformance.m run this as a dependency.

%% Setup

% Load Car Configuration
car = CarV4();
wheelbase = car.Wheelbase;
trackwidth = car.Trackwidth;
C = car.Cfactor;
wheelRadius = car.WheelRadius;

% SolidWorks Reference Points (converted to ISO coordinates)
swUBJstatic = [-185 610 -15.04];
swLBJstatic = [-80 175 4.96];
tp.UBJ = sw2iso(swUBJstatic);
tp.LBJ = sw2iso(swLBJstatic);

% Flexible Steering Linkage Parameters
ERz = 695.58;                  % Extension rod Z height [mm]
SAz0 = 650;                 % Steering arm Z height [mm]
setback = 520;           % X distance, ER axis to WC [mm]
ERconnectionLen = 30;       % Extension rod connection length [mm]
%SAvec = [0 0];             % Steering arm offset (from extended KP axis) [X-inset, Y-inset] [mm]
assert(exist('SAvec', 'var'), "Steering parameters missing :/");

%% Linkage Calculation

% Tire Patch Reference Frame Points
tp.KP = getPtOnAxis(tp.UBJ, tp.LBJ, wheelRadius);
tp.spindle = [0; tp.KP(2); tp.KP(3)];
tp.KP_SA = getPtOnAxis(tp.UBJ, tp.LBJ, SAz0);   % place to apply SAvec from
tp.SA_TR = tp.KP_SA + [SAvec(1); SAvec(2); 0];  % steering arm-tie rod node

% Static Nodes (Port Side)
ERaxisFun = @(dy) [wheelbase - setback; 0.5*ERconnectionLen; ERz] + dy*[0;1;0];
sNodes.TP = [wheelbase; 0.5*trackwidth; 0];
sNodes.WC = sNodes.TP + [0; 0; wheelRadius];
sNodes.UBJ = sNodes.TP + tp.UBJ;
sNodes.LBJ = sNodes.TP + tp.LBJ;
sNodes.KP = sNodes.TP + tp.KP;
sNodes.spindle = sNodes.TP + tp.spindle;
sNodes.KP_SA = sNodes.TP + tp.KP_SA;
sNodes.SA_TR = sNodes.TP + tp.SA_TR;
sNodes.ER_TR = ERaxisFun(0);

% Static Nodes (Starboard Side) - Mirrored
ERaxisFunStarboard = @(dy) ERaxisFun(dy) - [0; ERconnectionLen; 0];
sNodesStarboard = structfun(@(v) [v(1); -v(2); v(3)], sNodes, UniformOutput=false);
sNodesStarboard.ER_TR = ERaxisFunStarboard(0);

% Conversion Functions
findRackPos = @(deg) C * deg/360;
findYokeAngle = @(pos) pos/C * 360;

% Calculate Linkage Limits
maxYoke = floor(abs(findLinkageLimit(sNodes, ERaxisFun, findYokeAngle))*10)/10;
maxYokeNeg = -maxYoke;

assert(abs(findRackPos(maxYoke)) < car.RPmaxTravel, ...
    "Linkage range of motion limited by rack travel!");

fprintf('Steering geometry loaded!\n');
