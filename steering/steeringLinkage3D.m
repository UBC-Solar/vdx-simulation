%[text]{"align":"center"} # **3D Steering Linkage Simulation**
%[text]{"align":"center"} 
%[text] This script allows changing of steering linkage parameters and reflects these changes in visuals and plots.
%[text] To view everything simultaneously, select 'Hide Code', and zoom out using `ctrl +` \[`-`\]
%[text] 
%[text] #### Initialization + UI
clear; format shortG;

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
SAvec = [-200 -100];        % [X-inset, Y-inset] %[control:editfield:45fc]{"position":[9,20]}
setback = 600;      % X dist, ER axis to WC %[control:editfield:0f8d]{"position":[11,14]}
ERconnectionLen = 350; %[control:editfield:8d5f]{"position":[19,22]}


% Slider ends (steering stops)
findRackPos = @(deg) C * deg/360;
findYokeAngle = @(pos) pos/C * 360;
RoM = 9999; %!!!
maxMovement = floor(min(car.RPmaxTravel, RoM)*1e6)/1e6;
maxMovementNeg = -maxMovement;
maxYoke = floor(findYokeAngle(maxMovement)*10)/10;
maxYokeNeg = -maxYoke;

yokeAngle = -1 *0; % shown sign is flipped %[control:slider:0bfe]{"position":[17,18]}
computeSupplementary = true;
forceFit = false;
%[text] 
%[text] #### Calculation
tp.KP = getPtOnAxis(tp.UBJ, tp.LBJ, wheelRadius);
tp.spindle = [0; tp.KP(2); tp.KP(3)];
tp.KP_SA = getPtOnAxis(tp.UBJ, tp.LBJ, SAz0);   % place to apply SAvec from
tp.SA_TR = tp.KP_SA + [SAvec(1); SAvec(2); 0];  % str arm-tie rod node



ERaxisFun = @(dy) [0.5*wheelbase - setback; 0.5*ERconnectionLen; ERz] + dy*[0;1;0];

sNodes.TP = [0.5*wheelbase; 0.5*trackwidth; 0];
sNodes.WC = sNodes.TP + [0; 0; wheelRadius];
sNodes.UBJ = sNodes.TP + tp.UBJ;
sNodes.LBJ = sNodes.TP + tp.LBJ;
sNodes.KP = sNodes.TP + tp.KP;
sNodes.spindle = sNodes.TP + tp.spindle;
sNodes.KP_SA = sNodes.TP + tp.KP_SA;
sNodes.SA_TR = sNodes.TP + tp.SA_TR;
sNodes.ER_TR = ERaxisFun(0);

% Main
[dNodes, spin, SApathFun] = solveNodes(sNodes, ERaxisFun, findRackPos(yokeAngle));
renderLinkage(dNodes, SApathFun);
%[text] 
%[text] #### Solve Dynamic Linkage Positions
function [dNodes, spin, SApathFun] = solveNodes(sNodes, ERaxisFun, rackShift)
% Contraints for geometry solve
tieRodLen = norm(sNodes.ER_TR - sNodes.SA_TR);
KPaxisFun = @(t) sNodes.LBJ + t*((sNodes.UBJ - sNodes.LBJ) / norm(sNodes.UBJ - sNodes.LBJ));
SApathFun = getRevolvePath(KPaxisFun, sNodes.SA_TR);

switch sign(rackShift)
    case 0
        dNodes = sNodes;
        spin = 0;
        return
    case 1
        pathDomain = [-pi 0];  % steering arm swings CW
    case -1
        pathDomain = [0 pi];   % steering arm swings CCW
end

% Solve contraints

dNodes.ER_TR = ERaxisFun(rackShift);

% Solve
fun = @(theta) tieRodLen - norm(SApathFun(theta) - dNodes.ER_TR);
spin = fzero(fun, pathDomain);  % [rad]

dNodes.SA_TR = SApathFun(spin);
dNodes.TP = rotAboutAxis(KPaxisFun, sNodes.TP, spin);
dNodes.WC = rotAboutAxis(KPaxisFun, sNodes.WC, spin);
dNodes.spindle = rotAboutAxis(KPaxisFun, sNodes.spindle, spin);
dNodes.TP = rotAboutAxis(KPaxisFun, sNodes.TP, spin);

dNodes.UBJ = sNodes.UBJ;
dNodes.LBJ = sNodes.LBJ;
dNodes.KP = sNodes.KP;
dNodes.KP_SA = sNodes.KP_SA;
end
%[text] 
%[text] #### Render Linkage Diagram
function renderLinkage(nodes, SApathFun)
linkage = [nodes.TP, nodes.WC, nodes.spindle, nodes.KP, nodes.KP_SA, nodes.SA_TR, nodes.ER_TR];  % linkage members
balljoint = [nodes.UBJ nodes.LBJ];                                              % kingpin
ringThetas = linspace(-pi, pi, 64); ring = zeros(3, length(ringThetas));        % steering arm path
for i=1:length(ringThetas)
    ring(:,i) = SApathFun(ringThetas(i));
end

close all
figure(Visible='on') % pop out
clf

plot3(balljoint(1,:), balljoint(2,:), balljoint(3,:), 'bx--');
hold on
plot3(ring(1,:), ring(2,:), ring(3,:), 'm:')
plot3(linkage(1,:), linkage(2,:), linkage(3,:), 'r', LineWidth=2)
renderTires(nodes.TP, nodes.WC, nodes.spindle)
axis equal
xticks(''); yticks(''); zticks('');
xlabel('X (longitudinal)'); ylabel('Y (lateral)'); zlabel('Z (vertical)');
end

function renderTires(tirePatch, wheelCenter, spindleEnd)
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
    fill3(pts(1,:), pts(2,:), pts(3,:), 'k', 'FaceAlpha', 0.1);
end
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
