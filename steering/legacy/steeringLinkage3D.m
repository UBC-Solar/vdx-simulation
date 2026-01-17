%[text]{"align":"center"} # **3D Steering Linkage Simulation**
%[text]{"align":"center"} 
%[text] This script allows changing of steering linkage parameters and reflects these changes in visuals and plots.
%[text] To view everything simultaneously, select 'Hide Code', and zoom out using `ctrl +` \[`-`\]
%[text] 
%%
%[text] #### Initialization + UI
clear; format shortG; close all;

% Constants
car = CarV4();
wheelbase = car.Wheelbase;
trackwidth = car.Trackwidth;
C = car.Cfactor;
wheelRadius = car.WheelRadius;

% Solidworks reference points
swUBJ = [-185 625 -28.5];
swLBJ = [-60 175 -9];
tp.UBJ = sw2iso(swUBJ);
tp.LBJ = sw2iso(swLBJ);

% Flexible Parameters
ERz = 580;                  % Extension rod Z height %[control:editfield:97ad]{"position":[7,10]}
SAz0 = 580;                 % Steering arm Z height %[control:editfield:92b7]{"position":[8,11]}
SAvec = [-170 -65];         % Steering arm offset [X-inset, Y-inset] %[control:editfield:45fc]{"position":[9,19]}
setback = 404;              % X distance, ER axis to WC %[control:editfield:0f8d]{"position":[11,14]}
ERconnectionLen = 150;      % Extension rod connection length %[control:editfield:8d5f]{"position":[19,22]}
%[text] 
%%
%[text] #### Core Linkage Calculation

% Tire patch reference frame points
tp.KP = getPtOnAxis(tp.UBJ, tp.LBJ, wheelRadius);
tp.spindle = [0; tp.KP(2); tp.KP(3)];
tp.KP_SA = getPtOnAxis(tp.UBJ, tp.LBJ, SAz0);   % Place to apply SAvec from
tp.SA_TR = tp.KP_SA + [SAvec(1); SAvec(2); 0];  % Steering arm-tie rod node

% Static nodes (port side)
sNodes.TP = [wheelbase; 0.5*trackwidth; 0];
sNodes.WC = sNodes.TP + [0; 0; wheelRadius];
sNodes.UBJ = sNodes.TP + tp.UBJ;
sNodes.LBJ = sNodes.TP + tp.LBJ;
sNodes.KP = sNodes.TP + tp.KP;
sNodes.spindle = sNodes.TP + tp.spindle;
sNodes.KP_SA = sNodes.TP + tp.KP_SA;
sNodes.SA_TR = sNodes.TP + tp.SA_TR;

% Mirror to starboard side
sNodesStarboard = structfun(@(v) [v(1); -v(2); v(3)], sNodes, UniformOutput=false);

ERaxisFun = @(dy) [wheelbase - setback; 0.5*ERconnectionLen; ERz] + dy*[0;1;0];
ERaxisFunStarboard = @(dy) ERaxisFun(dy) - [0; ERconnectionLen; 0];
sNodes.ER_TR = ERaxisFun(0);
sNodesStarboard.ER_TR = ERaxisFunStarboard(0);

% Calculate linkage limits (steering stops)
findRackPos = @(deg) C * deg/360;
findYokeAngle = @(pos) pos/C * 360;

maxYoke = floor(abs(findLinkageLimit(sNodes, ERaxisFun, findRackPos, findYokeAngle))*10)/10;
maxYokeNeg = -maxYoke;

assert(abs(findRackPos(maxYoke)) < car.RPmaxTravel, "rack can't move to extend position");

% Current yoke position
yokeAngle = -1 *96.1;  % shown sign is flipped %[control:slider:0bfe]{"position":[17,21]}
turningDir = - sign(yokeAngle);
computeSupplementary = true;
forceFit = false;

% Solve linkage geometry for current yoke position
[nodesPort, spinPort, SApathFunPort] = ...
    solveNodes(car, sNodes, ERaxisFun, findRackPos(yokeAngle), 1);
[nodesStarboard, spinStarboard, SApathFunStarboard] = ...
    solveNodes(car, sNodesStarboard, ERaxisFunStarboard, findRackPos(yokeAngle), -1);

% Calculate steer angles
deltaL = getDelta(nodesPort.WC, nodesPort.spindle);
deltaR = getDelta(nodesStarboard.WC, nodesStarboard.spindle);
%[text] 
%%
%[text] #### 3D Linkage Visualization
figure
clf; view(3); hold on; axis equal;
xticks([]); yticks([]); zticks([]);
xlabel('X (longitudinal)'); ylabel('Y (lateral)'); zlabel('Z (vertical)');

% Core linkage
renderLinkage(nodesPort, SApathFunPort);                       % Port side linkage
renderLinkage(nodesStarboard, SApathFunStarboard);             % Starboard side linkage
connectPts([nodesPort.ER_TR, nodesStarboard.ER_TR], 'm-', LineWidth=2) % Extension rod

% Auxiliary points
plot3(wheelbase-setback, 0, ERz, 'k|', LineWidth=3)            % Extension rod centermark
connectPts([nodesPort.TPrigid, nodesStarboard.TPrigid], 'r.')  % Original contact points
connectPts([nodesPort.TP, nodesStarboard.TP], 'k.')            % New contact points
addFloorPlane(gca)                                             % Gray floor
%[text] #### 
%%
%[text] #### Ackermann Analysis
figure; clf; hold on; axis equal;
box on; xticks([]); yticks([]);

% Draw rear and front tires
drawTire(car, [0, 0.5*trackwidth], 0);
drawTire(car, [0, -0.5*trackwidth], 0);
drawTire(car, nodesPort.TP, deltaL);
drawTire(car, nodesStarboard.TP, deltaR);

% Store zoomed view limits
zoom(0.5);
zoomedX = xlim; zoomedY = ylim;

% Draw instantaneous center lines
if turningDir ~= 0
    interceptL = getICintercept(nodesPort.TP, deltaL);
    interceptR = getICintercept(nodesStarboard.TP, deltaR);
    interceptFar = turningDir * max(abs(interceptL), abs(interceptR));

    connectPts([0, 0; -0.5*turningDir*trackwidth, interceptFar], '--', LineWidth=0.5, Color='#888');
    connectPts([nodesPort.TP(1:2), [0; interceptL]], '--', LineWidth=0.5, Color='#F88');
    connectPts([nodesStarboard.TP(1:2), [0; interceptR]], '--', LineWidth=0.5, Color='#8F8');
end

% Zoom control
zoomIn = false; %[control:statebutton:43a4]{"position":[10,15]}
switch zoomIn
    case true
        xlim(zoomedX);
        ylim(zoomedY);
    case false
        axis padded;
        axis equal;
end
%[text] 
%%
%[text] ### Munro's Method Analysis (setup dataframe)

% Initialize sweep parameters
df = table();
extent = 90;
n = 128;

twoSided = true; %[control:statebutton:4b8e]{"position":[12,16]}

df.yokePos = linspace(twoSided*maxYokeNeg, maxYoke, n);
df.rackPos = -findRackPos(df.yokePos); %% TODO: Interim solution, verify sign convention

% Solve port side nodes
df.spinPort = zeros(1, n);
for i = 1:n
    [dfNodesPort(i), df.spinPort(i), ~] = solveNodes(car, sNodes, ERaxisFun, df.rackPos(i), 1);
end

% Solve starboard side nodes
df.spinStarboard = zeros(1, n);
for i = 1:n
    [dfNodesStarboard(i), df.spinStarboard(i), ~] = ...
        solveNodes(car, sNodesStarboard, ERaxisFunStarboard, df.rackPos(i), -1);
end

% Calculate steer angles
df.deltaL = rad2deg(arrayfun(@(s) getDelta(s.WC, s.spindle), dfNodesPort));
df.deltaR = rad2deg(arrayfun(@(s) getDelta(s.WC, s.spindle), dfNodesStarboard));
df.deltaEff = acotd((cotd(df.deltaL) + cotd(df.deltaR)) ./ 2);

% Determine outer/inner wheel based on turn direction
df.deltaOuter = zeros(1, n);
df.deltaInner = zeros(1, n);
for i = 1:n
    if df.rackPos(i) > 0  % right turn
        df.deltaOuter(i) = df.deltaL(i);
        df.deltaInner(i) = df.deltaR(i);
    else  % left turn
        df.deltaOuter(i) = df.deltaR(i);
        df.deltaInner(i) = df.deltaL(i);
    end
end

% Calculate Ackermann geometry metrics
df.deltaInnerIdeal = sign(df.deltaOuter) .* acotd(cotd(abs(df.deltaOuter)) - trackwidth/wheelbase);
df.innerError = (df.deltaInner - df.deltaInnerIdeal) ./ df.deltaInnerIdeal;
df.dynamicToe = abs(df.deltaOuter) - abs(df.deltaInner);
df.dynamicToeIdeal = abs(df.deltaOuter) - abs(df.deltaInnerIdeal);  %% TODO: Room for improvement in ideal calculation

% Calculate instantaneous centers and turn radii
df.intL = zeros(1, n);
df.intR = zeros(1, n);
df.radiusL = zeros(1, n);
df.radiusR = zeros(1, n);
for i = 1:n
    if abs(df.deltaL(i)) > eps
        df.intL(i) = getICintercept(dfNodesPort(i).TP, deg2rad(df.deltaL(i)));
        df.radiusL(i) = abs(dfNodesPort(i).TP(1) / sind(df.deltaL(i)));
    else
        df.intL(i) = inf;
        df.radiusL(i) = inf;
    end
    if abs(df.deltaR(i)) > eps
        df.intR(i) = getICintercept(dfNodesStarboard(i).TP, deg2rad(df.deltaR(i)));
        df.radiusR(i) = abs(dfNodesStarboard(i).TP(1) / sind(df.deltaR(i)));
    else
        df.intR(i) = inf;
        df.radiusR(i) = inf;
    end
end

% Calculate combined turning metrics
df.radiusEff = abs(wheelbase ./ sind(df.deltaEff));
df.ICdist = -(df.intL - df.intR);  % inner to outer
df.speedForLateral1G = sqrt(Phys.g * abs(df.radiusEff/1e3));

% Calculate parasitic camber
df.parasiticCamberL = zeros(1, n);
df.parasiticCamberR = zeros(1, n);
for i = 1:n
    hVecL = dfNodesPort(i).TP(1:2) - dfNodesPort(i).WC(1:2);
    hVecR = dfNodesStarboard(i).TP(1:2) - dfNodesStarboard(i).WC(1:2);

    distOverL = sign(hVecL(1)) * norm(hVecL);
    distOverR = sign(hVecR(1)) * norm(hVecR);

    heightL = dfNodesPort(i).WC(3) - dfNodesPort(i).TP(3);
    heightR = dfNodesStarboard(i).WC(3) - dfNodesStarboard(i).TP(3);

    df.parasiticCamberL(i) = atand(distOverL/heightL);
    df.parasiticCamberR(i) = atand(distOverR/heightR);
end


df.turnCentre = sign(df.deltaEff) .* sqrt(df.radiusEff.^2 - wheelbase.^2); %% TODO: Confirm signs

df.betaMotionL = zeros(1, n);
df.betaMotionR = zeros(1, n);
for i = 1:n
    df.betaMotionL(i) = atand(dfNodesPort(i).TP(1) ./ (df.turnCentre(i) - dfNodesPort(i).TP(2)));
    df.betaMotionR(i) = atand(dfNodesStarboard(i).TP(1) ./ (df.turnCentre(i) - dfNodesStarboard(i).TP(2)));
end

df.inducedAlphaL = df.deltaL - df.betaMotionL;
df.inducedAlphaR = df.deltaR - df.betaMotionR;

%TODO: figure; hold on
%plot(df.yokePos, df.turnCentre);

df.inducedPowerLossL = arrayfun(@(a) slip2latForce(deg2rad(abs(a)), 0.25*car.Weight, car), df.inducedAlphaL);
df.inducedPowerLossR = arrayfun(@(a) slip2latForce(deg2rad(abs(a)), 0.25*car.Weight, car), df.inducedAlphaR);
df.inducedPowerLossTot = df.inducedPowerLossL + df.inducedPowerLossR;



%[text] 
%[text] ### Munro's Method Plots

% Set reference yoke positions for plots
switch twoSided
    case true
        referenceYoke = [-75, 75];
    case false
        referenceYoke = 75;
end

% Turning radius plot
figure; hold on
plot(df.yokePos, df.radiusL/1e3, 'r');
plot(df.yokePos, df.radiusR/1e3, 'g');
plot(df.yokePos, df.radiusEff/1e3, 'y');
yline(12, 'k--');
yline(15/2, 'c--')
xline(referenceYoke, '--');
ylim([0 60]);
xlabel('Yoke Position [°]'); ylabel('Corner Radius (@ front wheel) [m]');
legend('Left Wheel', 'Right Wheel', 'Effective', 'Tightest Road Radius', 'U-turn Requirement');
title('Turning Tightness');

% Induced power loss plot
figure; hold on
plot(df.yokePos, df.inducedPowerLossTot, 'b');
plot(df.yokePos, df.inducedPowerLossL, 'r');
plot(df.yokePos, df.inducedPowerLossR, 'g');
xline(referenceYoke, '--');
xlabel('Yoke Position [°]'); ylabel('Power @ 10m/s [W]');
legend('Current Configuration');
title('Geometry-Induced Slip Power Losses');

% IC distance plot
figure
plot(df.yokePos, df.ICdist, 'b', LineWidth=2);
yline(0, 'm--', LineWidth=2);
yline(-trackwidth, '--', LineWidth=2);
xline(referenceYoke, '--');
ylim([min([df.ICdist, -trackwidth]) - 100, max([df.ICdist, 0]) + 100]);

regions.xlim = xlim;
regions.ylim = ylim;
regions.x = [regions.xlim(1), regions.xlim(2), regions.xlim(2), regions.xlim(1)];
regions.yAA = [-trackwidth, -trackwidth, regions.ylim(1), regions.ylim(1)];
regions.yLA = [0, 0, -trackwidth, -trackwidth];
regions.yPA = [regions.ylim(2), regions.ylim(2), 0, 0];

patch(regions.x, regions.yAA, 'r', FaceAlpha=0.2, EdgeColor='none');
patch(regions.x, regions.yLA, 'c', FaceAlpha=0.2, EdgeColor='none');
patch(regions.x, regions.yPA, 'y', FaceAlpha=0.2, EdgeColor='none');

xlim tight;
xlabel('Yoke Position [°]'); ylabel('IC Distance [mm]');
text('String', '\it{outside whl too straight}', 'Units', 'normalized', 'Position', [1.07, 0.9]);
legend('Current Geometry', 'True Ackermann', 'Parallel Steer', ...
    '"Anti-Ackermann"', '"Lazy Ackermann"', '"Pro-Ackermann"', location='eastoutside');
text('String', '\it{outside whl too toed}', 'Units', 'normalized', 'Position', [1.07, 0.1]);
title('Inner Tire IC ➡ Outer Tire IC');

% Parasitic camber plot
figure; hold on
plot(df.yokePos, df.parasiticCamberL, 'r');
plot(df.yokePos, df.parasiticCamberR, 'g');
xline(referenceYoke, '--');
xlabel('Yoke Position [°]'); ylabel('Tire Camber [°]');
legend('Left Wheel', 'Right Wheel');
title('Steering-Induced Camber');

% Induced wheel slip angle plot
figure; hold on
plot(df.yokePos, df.inducedAlphaL, 'r');
plot(df.yokePos, df.inducedAlphaR, 'g');
xline(referenceYoke, '--');
xlabel('Yoke Position [°]'); ylabel('Tire Slip [°]');
legend('Left Wheel', 'Right Wheel');
title('Steering-Induced Wheel \alpha');

% Dynamic toe plot
figure; hold on
plot(df.yokePos, df.dynamicToe, 'b');
plot(df.yokePos, df.dynamicToeIdeal, 'm');
xline(referenceYoke, '--');
xlabel('Yoke Position [°]'); ylabel('Dynamic Toe [°]');
text('String', '\it{Toe in (+)}', Position=[0.5, 0.9], Units='normalized', HorizontalAlignment='center');
text('String', '\it{Toe out (-)}', Position=[0.5, 0.1], Units='normalized', HorizontalAlignment='center');
legend('Current Configuration', 'True Ackermann');
title('Dynamic Toe');

% Pure rolling closeness plot
figure
plot(df.deltaOuter, df.innerError, 'b');
yline(0, 'm--');
xlabel('Outer Steer Angle [°]'); ylabel('% Error in Inner Steer Angle');
legend('Current Configuration', 'True Ackermann');
title('Pure Rolling Closeness');

% Wheel steer angle plot
figure; hold on
plot(df.yokePos, df.deltaL, 'r');
plot(df.yokePos, df.deltaR, 'g');
plot(df.yokePos, df.deltaEff, 'y');
plot(df.yokePos, df.betaMotionL, 'm'); %%%
plot(df.yokePos, df.betaMotionR, 'b'); %%%
xline(referenceYoke, '--');
xlabel('Yoke Position [°]'); ylabel('Steer Angle [°]');
legend('\delta Left Wheel', '\delta Right Wheel', '\delta_{eff}', '\beta Left Wheel', '\beta Right Wheel');
title('Wheel Steer');

% Lateral acceleration plot
figure
plot(df.radiusEff/1e3, df.speedForLateral1G, 'k');
xlabel('Cornering Radius [m]'); ylabel('1G Corner Maxspeed [m/s]');
xlim([0 100]);
title('a_c=v^2/r Result');
%[text] 
%%
%[text] #### Calculate Linkage Overextension Limit

function maxYoke = findLinkageLimit(sNodes, ERaxisFun, findRackPos, findYokeAngle)
% findLinkageLimit - Find maximum yoke angle before tie rod overextension
%   Uses port side geometry; result is symmetric for starboard

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
%[text] 
%%
%[text] #### Solve Dynamic Linkage Positions

function [dNodes, spin, SApathFun] = solveNodes(car, sNodes, ERaxisFun, rackShift, isPort)

% Contraints for geometry solve
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
            error("No upright orientation satifies %f mm rack position!\nLinkage overextended!"...
                , rackShift)
        end
        pathDomain = [0 thetaPositive];
end

% debug
%tt = linspace(-pi, pi, 256);
%plot(tt, arrayfun(@(t) slack(t),tt));
%xline(pathDomain, 'k--')
%yline(0, 'r--')

% Solve
spin = isPort*fzero(slack, pathDomain);  % [rad]

dNodes.SA_TR = SApathFun(spin);
dNodes.TPrigid = rotAboutAxis(KPaxisFun, sNodes.TP, spin);
dNodes.WC = rotAboutAxis(KPaxisFun, sNodes.WC, spin);
dNodes.spindle = rotAboutAxis(KPaxisFun, sNodes.spindle, spin);

dNodes.UBJ = sNodes.UBJ;
dNodes.LBJ = sNodes.LBJ;
dNodes.KP = sNodes.KP;
dNodes.KP_SA = sNodes.KP_SA;

dNodes.TP = ringPointMinZ(dNodes.WC, dNodes.WC - dNodes.spindle, car.WheelRadius);
end
%[text] 
%%
%[text] #### Determine Wheel Heading
function delta = getDelta(wheelCenterPt, spindlePt)
spindleVec = wheelCenterPt - spindlePt;
up = [0; 0; 1];
fwd = [1; 0; 0];

heading = cross(spindleVec, up);
heading = heading/norm(heading);
if heading(1) < 0
    heading = - heading;
end

angleMag = acos(dot(heading, fwd));
if angleMag <= eps
    delta = 0;
else
    angleDir = sign(dot(cross(fwd, heading), up));  % Z component of cross
    delta = angleDir * angleMag;
end
end
%[text] 
%%
%[text] #### Render Linkage Diagram
function renderLinkage(nodes, SApathFun)
ringThetas = linspace(-pi, pi, 64); ring = zeros(3, length(ringThetas));
for i=1:length(ringThetas)
    ring(:,i) = SApathFun(ringThetas(i));
end

connectPts([nodes.WC, nodes.spindle], 'r-', LineWidth=2)  % Spindle/tire
connectPts([nodes.spindle, nodes.KP, nodes.KP_SA, nodes.SA_TR], 'r-', LineWidth=2);  % Upright
connectPts([nodes.SA_TR, nodes.ER_TR], 'b-', LineWidth=2);  % Tie rod
connectPts([nodes.UBJ, nodes.LBJ], 'kx--');   % Kingpin axis
connectPts(ring, 'g:')                        % Steering arm path

renderTire(nodes.TP, nodes.WC, nodes.spindle)
end

function renderTire(tirePatch, wheelCenter, spindleEnd)
% renderTire - Render tire geometry as swept surface
arguments
    tirePatch   (3,:) double
    wheelCenter (3,:) double
    spindleEnd  (3,:) double
end
assert(isequal(size(tirePatch), size(wheelCenter), size(spindleEnd)))

thetas = linspace(0, 2*pi, 128);

for num=1:size(tirePatch, 2)
    v = @(t) spindleEnd(:,num) + t*(wheelCenter(:,num) - spindleEnd(:,num));
    tp = tirePatch(:,num);
    sweep = getRevolvePath(v, tp);
    pts = zeros(3, numel(thetas));
    for k = 1:numel(thetas)
        pts(:,k) = sweep(thetas(k));
    end
    fill3(pts(1,:), pts(2,:), pts(3,:), 'k', FaceAlpha=0.05);
end
end

function addFloorPlane(fig)
% addFloorPlane - Add dynamic floor plane to 3D plot
[X, Y, Z] = getCorners(fig);
hPlane = fill3(fig, X, Y, Z, 'k', FaceAlpha=0.2);
addlistener(fig, 'XLim', PostSet=@updatePlane);
addlistener(fig, 'YLim', PostSet=@updatePlane);

    function [X, Y, Z] = getCorners(fig)
        xl = fig.XLim;
        yl = fig.YLim;
        X = [xl(1) xl(2) xl(2) xl(1)];
        Y = [yl(1) yl(1) yl(2) yl(2)];
        Z = zeros(1,4);
    end

    function updatePlane(~,~)
        [X, Y, Z] = getCorners(fig);
        hPlane.XData = X;
        hPlane.YData = Y;
        hPlane.ZData = zeros(1,4);
    end
end

function connectPts(matrix, LineSpec, opts)
% connectPts - Plot connected points in 2D or 3D
arguments
    matrix (:,:) double
    LineSpec string
    opts.LineWidth (1,1) {mustBeNumeric} = 1
    opts.Color = []
end

if any(ismember(char(LineSpec), ['r','g','b','c','m','y','k','w']))
    colorArg = {};
elseif opts.Color
    colorArg = {'Color', opts.Color};
else
    colorArg = {'Color', gca().ColorOrder(gca().ColorOrderIndex, :)};
end

switch size(matrix,1)
    case 2
        plot(matrix(1,:), matrix(2,:), LineSpec, colorArg{:}, LineWidth=opts.LineWidth);
    case 3
        plot3(matrix(1,:), matrix(2,:), matrix(3,:), LineSpec, colorArg{:}, LineWidth=opts.LineWidth);
    otherwise
        error('huh')
end
end
%[text] 
%%
%[text] #### Ackermann Visualization

function drawTire(car, pos, delta)
% drawTire - Draw 2D tire footprint for Ackermann visualization
t = car.WheelCurvature; r = car.WheelRadius;
rotM = [cos(delta), -sin(delta); sin(delta), cos(delta)];
offset = rotM*[r, -r, -r, r, r;
    t, t, -t, -t, t];
plot(pos(1) + offset(1,:), pos(2) + offset(2,:), 'k');
end

%[text] #### Instantaneous Center Intercept

function intercept = getICintercept(tirePos, steerAngle)
% getICintercept - Calculate lateral distance to instantaneous center
%   Returns lateral distance to IC along vehicle X-axis
intercept = tirePos(2) + tirePos(1)*cot(steerAngle);
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"hidecode","rightPanelPercent":30.8}
%---
%[control:editfield:97ad]
%   data: {"defaultValue":0,"label":"Extension rod Z height","run":"SectionToEnd","valueType":"Double"}
%---
%[control:editfield:92b7]
%   data: {"defaultValue":0,"label":"Steering arm Z height","run":"SectionToEnd","valueType":"Double"}
%---
%[control:editfield:45fc]
%   data: {"defaultValue":"0","label":"Steering arm offset [X-inset, Y-inset]","run":"SectionToEnd","valueType":"MATLAB code"}
%---
%[control:editfield:0f8d]
%   data: {"defaultValue":0,"label":"X distance, ER axis to WC","run":"SectionToEnd","valueType":"Double"}
%---
%[control:editfield:8d5f]
%   data: {"defaultValue":0,"label":"Extension rod connection length","run":"SectionToEnd","valueType":"Double"}
%---
%[control:slider:0bfe]
%   data: {"defaultValue":0,"label":"Yoke Position","max":96.1,"maxLinkedVariable":"maxYoke","min":-96.1,"minLinkedVariable":"maxYokeNeg","run":"AllSections","runOn":"ValueChanged","step":0.1}
%---
%[control:statebutton:43a4]
%   data: {"defaultValue":false,"label":"Toggle Plot Zoom","run":"Section"}
%---
%[control:statebutton:4b8e]
%   data: {"defaultValue":false,"label":"Toggle Symmetric Travel","run":"Section"}
%---
