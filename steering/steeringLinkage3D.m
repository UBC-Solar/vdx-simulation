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

% FL = car.FL
% UBJ = FL.ubj
% LBJ = FL.lbj

swUBJ = [-185 625 -28.5];
swLBJ = [-60 175 -9];
tp.UBJ = sw2iso(swUBJ);
tp.LBJ = sw2iso(swLBJ);

% Flexible Parameters
ERz = 500;          % extension rod Z %[control:editfield:97ad]{"position":[7,10]}
SAz0 = 500;         % steering arm Z      %[control:editfield:92b7]{"position":[8,11]}
SAvec = [-150 -40];        % [X-inset, Y-inset] %[control:editfield:45fc]{"position":[9,19]}
setback = 400;      % X dist, ER axis to WC %[control:editfield:0f8d]{"position":[11,14]}
ERconnectionLen = 350; %[control:editfield:8d5f]{"position":[19,22]}


% Slider ends (steering stops)
findRackPos = @(deg) C * deg/360;
findYokeAngle = @(pos) pos/C * 360;
RoM = 9999; %!!!
maxMovement = floor(min(car.RPmaxTravel, RoM)*1e6)/1e6;
maxMovementNeg = -maxMovement;
maxYoke = floor(findYokeAngle(maxMovement)*10)/10;
maxYokeNeg = -maxYoke;

yokeAngle = -1 *50;  % shown sign is flipped %[control:slider:0bfe]{"position":[17,19]}
turningDir = - sign(yokeAngle);
computeSupplementary = true;
forceFit = false;
%[text] 
%%
%[text] #### Core Linkage Calculation
tp.KP = getPtOnAxis(tp.UBJ, tp.LBJ, wheelRadius);
tp.spindle = [0; tp.KP(2); tp.KP(3)];
tp.KP_SA = getPtOnAxis(tp.UBJ, tp.LBJ, SAz0);   % place to apply SAvec from
tp.SA_TR = tp.KP_SA + [SAvec(1); SAvec(2); 0];  % str arm-tie rod node

sNodes.TP = [wheelbase; 0.5*trackwidth; 0];
sNodes.WC = sNodes.TP + [0; 0; wheelRadius];
sNodes.UBJ = sNodes.TP + tp.UBJ;
sNodes.LBJ = sNodes.TP + tp.LBJ;
sNodes.KP = sNodes.TP + tp.KP;
sNodes.spindle = sNodes.TP + tp.spindle;
sNodes.KP_SA = sNodes.TP + tp.KP_SA;
sNodes.SA_TR = sNodes.TP + tp.SA_TR;

sNodesStarboard = structfun(@(v) [v(1); -v(2); v(3)], sNodes, UniformOutput=false);  % simple mirror of port

ERaxisFun = @(dy) [wheelbase - setback; 0.5*ERconnectionLen; ERz] + dy*[0;1;0];
ERaxisFunStarboard = @(dy) ERaxisFun(dy) - [0; ERconnectionLen; 0];
sNodes.ER_TR = ERaxisFun(0);
sNodesStarboard.ER_TR = ERaxisFunStarboard(0);

[nodesPort, spinPort, SApathFunPort] = solveNodes(car, sNodes, ERaxisFun, findRackPos(yokeAngle), 1);
[nodesStarboard, spinStarboard, SApathFunStarboard] = ...
    solveNodes(car, sNodesStarboard, ERaxisFunStarboard, findRackPos(yokeAngle), -1);

deltaL = getDelta(nodesPort.WC, nodesPort.spindle);
deltaR = getDelta(nodesStarboard.WC, nodesStarboard.spindle);
%[text] 
%%
%[text] #### 3D Linkage Visualization
figure(Visible='on') % open in window
clf; view(3); hold on; axis equal;
xticks([]); yticks([]); zticks([]);
xlabel('X (longitudinal)'); ylabel('Y (lateral)'); zlabel('Z (vertical)');

% Core linkage
renderLinkage(nodesPort, SApathFunPort);                       % port side linkage
renderLinkage(nodesStarboard, SApathFunStarboard);             % right side linkage
connectPts([nodesPort.ER_TR, nodesStarboard.ER_TR], 'm-', LineWidth=2) % ex rod

% Auxiliary points
plot3(wheelbase-setback, 0, ERz, 'k|', LineWidth=3)            % ex rod centremark
connectPts([nodesPort.TPrigid, nodesStarboard.TPrigid], 'r.')  % original contact pts
connectPts([nodesPort.TP, nodesStarboard.TP], 'k.')            % new contact pts
addFloorPlane(gca)                                             % gray floor
%[text] #### 
%%
%[text] #### Ackermann Analysis
figure; clf; hold on; axis equal;
box on; xticks([]); yticks([]);

drawTire(car, [0, 0.5*trackwidth], 0);
drawTire(car, [0, -0.5*trackwidth], 0);
drawTire(car, nodesPort.TP, deltaL);
drawTire(car, nodesStarboard.TP, deltaR);

zoom(0.5);
zoomedX = xlim; zoomedY = ylim;

if turningDir ~= 0
    iczv = @(pos, angle) pos(2) + pos(1)*cot(angle);
    interceptL = iczv(nodesPort.TP, deltaL);
    interceptR = iczv(nodesStarboard.TP, deltaR);
    interceptFar = turningDir * max(abs(interceptL), abs(interceptR));
    
    connectPts([0, 0; -0.5*turningDir*trackwidth, interceptFar], '--', LineWidth=0.5, Color='#888')
    connectPts([nodesPort.TP(1:2), [0; interceptL]], '--', LineWidth=0.5, Color='#F88')
    connectPts([nodesStarboard.TP(1:2), [0; interceptR]], '--', LineWidth=0.5, Color='#8F8')
end

zoomIn = true; %[control:statebutton:43a4]{"position":[10,14]}
switch zoomIn
    case true
        xlim(zoomedX); ylim(zoomedY);
    case false
        axis padded
        axis equal
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
        spin = 0;
        return
    case 1  % steering arm swings CW (rack is approaching)
        thetaNegative = fminbnd(@(t) slack(t), -pi, 0);
        pathDomain = [thetaNegative 0];
    case -1  % steering arm swings CCW (rack is retreating)
        thetaPositive = fminbnd(@(t) -slack(t), 0, pi);  % closest pt on arc to ER_TR
        if slack(thetaPositive) < 0
            error("No upright orientation satifies %f mm rack position!\nLinkage overextended!", rackShift)
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

connectPts([nodes.WC, nodes.spindle], 'r-', LineWidth=2)  % spindle/tire
connectPts([nodes.spindle, nodes.KP, nodes.KP_SA, nodes.SA_TR], 'r-', LineWidth=2);  % upright
connectPts([nodes.SA_TR, nodes.ER_TR], 'b-', LineWidth=2);  % tie rod
connectPts([nodes.UBJ, nodes.LBJ], 'kx--');   % kingpin
connectPts(ring, 'g:')                        % steering arm path

renderTire(nodes.TP, nodes.WC, nodes.spindle)
end


function renderTire(tirePatch, wheelCenter, spindleEnd)
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
    t = car.WheelCurvature; r = car.WheelRadius;
    rotM = [cos(delta), -sin(delta); sin(delta), cos(delta)];
    offset = rotM*[r, -r, -r, r, r;
                  t, t, -t, -t, t];
    plot(pos(1) + offset(1,:), pos(2) + offset(2,:), 'k');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":26.5}
%---
%[control:editfield:97ad]
%   data: {"defaultValue":0,"label":"setback","run":"SectionToEnd","valueType":"Double"}
%---
%[control:editfield:92b7]
%   data: {"defaultValue":0,"label":"setback","run":"SectionToEnd","valueType":"Double"}
%---
%[control:editfield:45fc]
%   data: {"defaultValue":"0","label":"connnection_length","run":"SectionToEnd","valueType":"MATLAB code"}
%---
%[control:editfield:0f8d]
%   data: {"defaultValue":0,"label":"setback","run":"SectionToEnd","valueType":"Double"}
%---
%[control:editfield:8d5f]
%   data: {"defaultValue":0,"label":"connnection_length","run":"SectionToEnd","valueType":"Double"}
%---
%[control:slider:0bfe]
%   data: {"defaultValue":0,"label":"Slider","max":353.5,"maxLinkedVariable":"max_yoke","min":-353.5,"minLinkedVariable":"max_yoke_neg","run":"AllSections","runOn":"ValueChanged","step":0.1}
%---
%[control:statebutton:43a4]
%   data: {"defaultValue":false,"label":"Toggle Plot Zoom","run":"SectionToEnd"}
%---
