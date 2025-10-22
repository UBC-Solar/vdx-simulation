%% MAIN SCRIPT TO CALULATE HARDPOINT FORCES %%
clear, close, clc, clf % clears workspace, closes figures, clears terminal, clear figure
x = 1; y = 2; z = 3; % for clarity

%%%%%% Set Loading Condition here! %%%%%%
%regs: 1g turn, 2g bump, 1g braking
bumpG = 2; %bump should >= 1 (no bump would be bumpG = 1 for static weight)
brakeG = 1;
cornerG = 1;
turnDirection = 1; %right
%turnDirection = -1; %left

if turnDirection == 1
    disp("***Turning Right***")
else
    disp("***Turning Left***")
end

%%%%%% Set side to compute and plot %%%%%
side = "Right"; % (coordiante driving primary side)
%side = "Left";
%side = "Both";

% (choose only Right or Left side to have forces table copied to clipboard to paste
% to google docs)


%%%%%% Vehicle Paramters %%%%%%

% Taken from SW VDX Skeleton and CG Estimate (April 23,2025)
%COM = [150.08743630, 434.37602167, 47.72138612]; % mm [x,y,z]
totalMass = 354.37; %kg
trackWidth = 1270; %mm
wheelBase = 2750; %mm
g = 9.81; % gravatiational acceleration


%%%%%% Set Plot View %%%%%%
sw_view('iso');
% type "help sw_view" in the command window for more information
% (or look at the sw_view.m file in suspension) 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT EDIT BELOW:
% Running Routines to Obtain Hardpoint Forces %

% Getting coordinates from google sheet SW Hardpoints tab of V4 Vehicle
% Dynamics sheet
run("getCoordinatesV3.m")

% Calculate Forces at Tire Patch
run("tirePatchForces.m");
disp("------Tire Patch Forces------")
forcesTP = {'f_FL_TP', 'f_FR_TP', 'f_RL_TP', 'f_RR_TP'};
displayVectorComponents(forcesTP)

disp("------Hardpoint Forces------")

if side == "Right" || side == "Both"

    disp("Right side Forces displyed in Newtons")

    % Primary Coordiante Side

    % Solve static forces system and plot suspension
    run("solveStaticSystem.m")
    
    displayVectorComponents(forceNames)
end

if side == "Left" || side == "Both"
    disp("Left side Forces displyed in Newtons")
    % Secondary Coordiante Side
    % negating the Y component
    for i = 1:numel(symbols)
        var = evalin('base', symbols(i));
        assignin('base',symbols(i),[-var(1), var(2), var(3)])
    end
    clear var;
    % Solve static forces system and plot suspension
    run("solveStaticSystem.m")
    displayVectorComponents(forceNames)
    
end

if side == "Left" || side == "Right"
    copyForceTableForGoogleDocs(forceNames, forces);
end

% Setting up figure
figure(1)
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Solar VDX Suspension');
subtitle(sprintf('Front %s Side(s)', side))
axis equal
hold on;
legend show;
grid on;

%% Plotting
% Drawing
scatter3(COM(1), COM(2), COM(3), 'rx', 'DisplayName', 'COM')

drawLink(pTR_out, pTR_in - pTR_out, "TR", "r")
drawLink(pU_LCA, pC_LCA_in - pU_LCA, "LCA_{in}", 'b')
drawLink(pU_LCA, pC_LCA_out - pU_LCA, "LCA_{out}", 'g')
drawLink(pU_UCA, pC_UCA_in - pU_UCA, "UCA_{in}", 'm')
drawLink(pU_UCA, pC_UCA_out - pU_UCA, "UCA_{out}", 'k')
drawLink(pUCA_PR, pR_PR - pUCA_PR, "PR", 'cyan')

% Plot wheel center and tire patch
tireRadius = norm(p_WC-p_TP);
scatter3(p_WC(1), p_WC(2), p_WC(3), 50, 'b', 'filled', 'DisplayName','WC')
scatter3(p_TP(1), p_TP(2), p_TP(3), 50, 'r', 'filled', 'DisplayName','TP')
drawCircle(p_WC, [1,0,0], tireRadius, "Tire", "k")

% Rocker Plotting
drawPoint(pR_C, 'pR_C', 'k')
drawPoint(pR_PR, 'pR_{PR}', 'o')
drawPoint(pR_S, 'pR_S', 'g')
rockerPoints = [pR_C; pR_PR; pR_S; pR_C];

% Connect rocker points with line
plot3(rockerPoints(:,x), rockerPoints(:,y), rockerPoints(:,z), 'DisplayName', 'Rocker')

hold off;

function displayVectorComponents(vectorNames)
    n = numel(vectorNames);
    components = zeros(n, 3);  % Preallocate matrix
    
    for i = 1:n
        components(i, :) = evalin('base', vectorNames{i});
    end
    
    % Create table
    disp(array2table(components, 'VariableNames', {'X', 'Y', 'Z'}, 'RowNames', vectorNames));
end

%% Forces to table functions
function copyForceTableForGoogleDocs(forceNames, forces)
% Copy tab-delimited force table with headers to clipboard for Google Docs

    if length(forceNames) ~= size(forces,1)
        error('forceNames length must match number of force rows');
    end

    % Header line using sprintf for tabs
    header = sprintf('Force Name\tX (N)\tY (N)\tZ (N)\n');

    rows = strings(length(forceNames),1);
    for i = 1:length(forceNames)
        xStr = formatNumWithCommas(forces(i,1));
        yStr = formatNumWithCommas(forces(i,2));
        zStr = formatNumWithCommas(forces(i,3));
        rows(i) = sprintf('%s\t%s\t%s\t%s', forceNames{i}, xStr, yStr, zStr);
    end

    % Combine header + rows with newline characters
    outputText = header + strjoin(rows, '\n');  

    % Convert to char for clipboard
    clipboard('copy', char(outputText));

    fprintf('Table copied to clipboard! Paste into Google Docs with Ctrl+V.\n');
end

function s = formatNumWithCommas(num)
    s = sprintf('%.2f', num);
    parts = split(s, '.');
    intPart = parts{1};
    decPart = parts{2};
    intPartWithCommas = insertCommas(intPart);
    s = [intPartWithCommas '.' decPart];
end

function s = insertCommas(str)
    n = length(str);
    s = '';
    count = 0;
    for i = n:-1:1
        s = [str(i) s];
        count = count + 1;
        if mod(count,3) == 0 && i ~= 1
            s = [',' s];
        end
    end
end

%% Draw Link Function
% Draws suspension link given base point and direction vector (including length). Also, input the plot display name
% and color of member.
function drawLink(basePoint, directionVector, name, color)
    LINE_WIDTH = 2;
    quiver3(basePoint(1), basePoint(2), basePoint(3), directionVector(1), directionVector(2), directionVector(3), 0, "DisplayName", name, "Color", color, "LineWidth", LINE_WIDTH)
end

%% Draw Circle Function
function drawCircle(center, normal, radius, name, color)
    % center: [x, y, z]
    % normal: normal vector to the plane
    % radius: circle radius
    % N: number of points (e.g., 100)
    % color: plot color (optional)

    N = 100;

    if nargin < 5
        color = 'b';
    end

    % Normalize normal vector
    n = normal / norm(normal);

    % Find two orthonormal vectors perpendicular to the normal
    if abs(n(3)) < 1
        v = [0, 0, 1];
    else
        v = [1, 0, 0];
    end
    x = cross(n, v); x = x / norm(x);
    y = cross(n, x); % already unit-length

    % Parametrize circle in local basis
    theta = linspace(0, 2*pi, N);
    circle = radius * (cos(theta)' * x + sin(theta)' * y) + center;

    % Plot
    plot3(circle(:,1), circle(:,2), circle(:,3), color, 'LineWidth', 2, "DisplayName", name);
end

%% Draw point function
function drawPoint(coordinateVector, name, color)
    scatter3(coordinateVector(1), coordinateVector(2), coordinateVector(3), color, 'filled', 'DisplayName', name)
end




