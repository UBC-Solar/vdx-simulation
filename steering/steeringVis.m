%% 3D Steering Linkage Visualization
% Interactive visualization of steering linkage geometry at a specific yoke position.
%
% This script produces:
%   1. 3D linkage diagram showing both port and starboard sides
%   2. 2D Ackermann analysis view with instantaneous center lines
%
% Run steeringGeometry.m first to initialize workspace variables.

%% Initialize Geometry
clear; format shortG; close all;

SAvec = [-160,  -45];         % Steering arm offset [X-inset, Y-inset] %[control:editfield:4126]{"position":[9,21]}
ERconnectionLen = 30;      % Extension rod connection length %[control:editfield:945c]{"position":[19,21]}
setback = 340;              % X distance, ER axis to WC %[control:editfield:0c75]{"position":[11,14]}

  %[control:button:85d2]{"position":[1,2]}

% Load shared geometry configuration
steeringGeometry;

%% Current Yoke Position
yokeAngle =0;  % Steer input [degrees], positive = right turn %[control:slider:679b]{"position":[12,13]}

turningDir = -sign(yokeAngle);

% Solve Linkage Geometry

susTravel = "droop" %[control:dropdown:1b3d]{"position":[13,20]}

tieRodLen = norm(sNodes.ER_TR - sNodes.SA_TR); % at static regardless
if ~strcmp(susTravel, "static")
    switch susTravel
        case "droop"
            jNodes.UBJ = sNodes.TP + sw2iso([-185 600 -29.23]);  %jounceNodes
            jNodes.LBJ = sNodes.TP + sw2iso([-60 150 -8.33]);
        case "bump"
            jNodes.UBJ = sNodes.TP + sw2iso([-185 674 -31.44]);
            jNodes.LBJ = sNodes.TP + sw2iso([-60 224.93 -6.3]);
    end
    vStatic = sNodes.UBJ([1,3]) - sNodes.LBJ([1,3]);
    vJounce = jNodes.UBJ([1,3]) - jNodes.LBJ([1,3]);

    theta = atan2(vJounce(2), vJounce(1)) - atan2(vStatic(2), vStatic(1));

    R2 = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    sCentroid = (sNodes.UBJ([1,3]) + sNodes.LBJ([1,3])) / 2;
    jCentroid = (jNodes.UBJ([1,3]) + jNodes.LBJ([1,3])) / 2;
    T2 = jCentroid - (R2 * sCentroid);

    M = eye(3); M([1 3], [1,3]) = R2;
    D = zeros(3,1); D([1,3]) = T2;
    jounce = @(p) (M * p(:) + D);

    sNodes.TP = jounce(sNodes.TP);
    sNodes.WC = jounce(sNodes.WC);
    sNodes.KP = jounce(sNodes.KP);
    sNodes.spindle = jounce(sNodes.spindle);
    sNodes.KP_SA = jounce(sNodes.KP_SA);
    sNodes.SA_TR = jounce(sNodes.SA_TR);
    sNodes.UBJ = jNodes.UBJ;
    sNodes.LBJ = jNodes.LBJ;

    sNodesStarboard = structfun(@(v) [v(1); -v(2); v(3)], sNodes, UniformOutput=false);
    sNodesStarboard.ER_TR = ERaxisFunStarboard(0);
end

[nodesPort, spinPort, SApathFunPort] = ...
    solveSteeringLinkage(car, sNodes, ERaxisFun, findRackPos(yokeAngle), 1, tieRodLen=tieRodLen);
[nodesStarboard, spinStarboard, SApathFunStarboard] = ...
    solveSteeringLinkage(car, sNodesStarboard, ERaxisFunStarboard, findRackPos(yokeAngle), -1, tieRodLen=tieRodLen);

% Calculate steer angles
deltaL = getDelta(nodesPort.WC, nodesPort.spindle);
deltaR = getDelta(nodesStarboard.WC, nodesStarboard.spindle);

fprintf('δL: %.2f°, δR: %.2f°\n', rad2deg(deltaL), rad2deg(deltaR));

%% 3D Linkage Visualization
figure('Name', '3D Steering Linkage', 'NumberTitle', 'off');
clf; view(3); hold on; axis equal;
xticks([]); yticks([]); zticks([]);
xlabel('X (longitudinal)'); ylabel('Y (lateral)'); zlabel('Z (vertical)');

% Core linkage
renderLinkage(nodesPort, SApathFunPort);                       % Port side linkage
renderLinkage(nodesStarboard, SApathFunStarboard);             % Starboard side linkage
connectPts([nodesPort.ER_TR, nodesStarboard.ER_TR], 'm-', LineWidth=2); % Extension rod

% Auxiliary points
plot3(wheelbase - setback, 0, ERz, 'k|', LineWidth=3);         % Extension rod centermark
connectPts([nodesPort.TPrigid, nodesStarboard.TPrigid], 'r.'); % Original contact points
connectPts([nodesPort.TP, nodesStarboard.TP], 'k.');           % New contact points
addFloorPlane(gca);                                            % Gray floor

title(sprintf('Steering Linkage @ %.1f° Yoke', yokeAngle));
%% 2D Ackermann Analysis
figure();
clf; hold on; axis equal;
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
zoomIn = true; %[control:statebutton:90f0]{"position":[10,14]}
switch zoomIn
    case true
        xlim(zoomedX);
        ylim(zoomedY);
    case false
        axis padded;
        axis equal;
end

title(sprintf('Ackermann View @ %.1f° Yoke', yokeAngle));

%% Print SA_TR Coordinates
fprintf('\nSteering Arm to Tie Rod Connection Points:\n');
fprintf('Portside Steering Arm Node [x,y,z] (ISO:8855):  [%.1f, %.1f, %.1f]\n', nodesPort.SA_TR);
fprintf('Portside Steering Arm Node [x,y,z] (SolidWorks): [%.1f, %.1f, %.1f]\n', iso2sw(nodesPort.SA_TR));
fprintf('Starboard Steering Arm Node [x,y,z] (ISO:8855):  [%.1f, %.1f, %.1f]\n', nodesStarboard.SA_TR);
fprintf('Starboard Steering Arm Node [x,y,z] (SolidWorks): [%.2f, %.1f, %.1f]\n', iso2sw(nodesStarboard.SA_TR));

function renderLinkage(nodes, SApathFun)
% renderLinkage - Render steering linkage in 3D
ringThetas = linspace(-pi, pi, 64);
ring = zeros(3, length(ringThetas));
for i = 1:length(ringThetas)
    ring(:,i) = SApathFun(ringThetas(i));
end

connectPts([nodes.WC, nodes.spindle], 'r-', LineWidth=2);          % Spindle/tire
connectPts([nodes.spindle, nodes.KP, nodes.KP_SA, nodes.SA_TR], 'r-', LineWidth=2);  % Upright
connectPts([nodes.SA_TR, nodes.ER_TR], 'b-', LineWidth=2);         % Tie rod
connectPts([nodes.UBJ, nodes.LBJ], 'kx--');                        % Kingpin axis
connectPts(ring, 'g:');                                            % Steering arm path

renderTire(nodes.TP, nodes.WC, nodes.spindle);
end

function renderTire(tirePatch, wheelCenter, spindleEnd)
% renderTire - Render tire geometry as swept surface
arguments
    tirePatch   (3,:) double
    wheelCenter (3,:) double
    spindleEnd  (3,:) double
end
assert(isequal(size(tirePatch), size(wheelCenter), size(spindleEnd)));

thetas = linspace(0, 2*pi, 128);

for num = 1:size(tirePatch, 2)
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
addlistener(fig, 'XLim', 'PostSet', @updatePlane);
addlistener(fig, 'YLim', 'PostSet', @updatePlane);

    function [X, Y, Z] = getCorners(fig)
        xl = fig.XLim;
        yl = fig.YLim;
        X = [xl(1) xl(2) xl(2) xl(1)];
        Y = [yl(1) yl(1) yl(2) yl(2)];
        Z = zeros(1, 4);
    end

    function updatePlane(~, ~)
        [X, Y, Z] = getCorners(fig);
        hPlane.XData = X;
        hPlane.YData = Y;
        hPlane.ZData = zeros(1, 4);
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
elseif ~isempty(opts.Color)
    colorArg = {'Color', opts.Color};
else
    colorArg = {'Color', gca().ColorOrder(gca().ColorOrderIndex, :)};
end

switch size(matrix, 1)
    case 2
        plot(matrix(1,:), matrix(2,:), LineSpec, colorArg{:}, LineWidth=opts.LineWidth);
    case 3
        plot3(matrix(1,:), matrix(2,:), matrix(3,:), LineSpec, colorArg{:}, LineWidth=opts.LineWidth);
    otherwise
        error('Matrix must have 2 or 3 rows');
end
end

function drawTire(car, pos, delta)
% drawTire - Draw 2D tire footprint for Ackermann visualization
t = car.WheelCurvature;
r = car.WheelRadius;
rotM = [cos(delta), -sin(delta); sin(delta), cos(delta)];
offset = rotM * [r, -r, -r, r, r; t, t, -t, -t, t];
plot(pos(1) + offset(1,:), pos(2) + offset(2,:), 'k');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
%[control:editfield:4126]
%   data: {"defaultValue":"0","label":"Steering arm offset [X-inset, Y-inset]","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:945c]
%   data: {"defaultValue":0,"label":"Extension rod connection length","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:0c75]
%   data: {"defaultValue":0,"label":"X distance, ER axis to WC","run":"Nothing","valueType":"Double"}
%---
%[control:button:85d2]
%   data: {"label":"Run","run":"AllSections"}
%---
%[control:slider:679b]
%   data: {"defaultValue":0,"label":"Yoke Position","max":169.7,"maxLinkedVariable":"maxYoke","min":-169.7,"minLinkedVariable":"maxYokeNeg","run":"AllSections","runOn":"ValueChanged","step":0.1}
%---
%[control:dropdown:1b3d]
%   data: {"defaultValue":"\"static\"","itemLabels":["bump","static","droop"],"items":["\"bump\"","\"static\"","\"droop\""],"label":"Drop down","run":"AllSections"}
%---
%[control:statebutton:90f0]
%   data: {"defaultValue":false,"label":"Toggle Plot Zoom","run":"Section"}
%---
