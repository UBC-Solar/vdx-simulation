%% MAIN SCRIPT TO CALCULATE HARDPOINT FORCES %%
clear, close, clc, % clears workspace, closes figures, clears terminal,

% Taken from SW VDX Skeleton and CG Estimate (April 23,2025)
totalMass = 354.37; %kg
wheelBase = 2750; %mm

%%%%%% Set Loading Condition here! %%%%%%
% regs: 2g bump, 1g brake, 1g corner
% Our new standard: 4g bump, 2g brake, 1g corner
loading.bump = 4; %bump should >= 1 (no bump would be bumpG = 1 for static weight)
loading.brake = 1;
loading.corner = 2;
loading.turnDirection = 1; %1:right, -1:left

%%%%%% Set side to compute and plot %%%%%
%side = "Right"; % (coordiante driving primary side)
side = "Left";
%side = "Both";

%%%%%% Set Plot View %%%%%%
%sw_view('right');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Display Turn Direction
if loading.turnDirection == 1
    disp("***Turning Right***")
elseif loading.turnDirection == -1
        disp("***Turning Left***")
else
    error("Invalid Turn Direction. Left: -1, Right: 1")
end

% Read coordinates from VDX Hardpoints spreadsheet and store in a struct, p
pFR = getCoordinates();

% Negate x component to optain left side hardpoint coordinates
pFL = structfun(@(p) [-p(1), p(2), p(3)], pFR, "UniformOutput", false);

% Compute forces at tire patch and store in a struct, f
f = computeTirePatchForces(pFR.COM, totalMass, wheelBase, 2*abs(pFR.TP(1)), loading);
disp("------Tire Patch Forces [N]------")
displayForceVectors(f)

disp("------ Front Hardpoint Forces------")
if side == "Right"
    disp("RIGHT SIDE:")
    F_FR = solveStaticSystemFUNC(pFR, f.FR);
    displayForceVectors(F_FR)

elseif side == "Left"
    disp("LEFT SIDE:")
    F_FL = solveStaticSystemFUNC(pFL, f.FL);
    displayForceVectors(F_FL)

end
% 
% % BUMPING %%%%%%%%%%%%%%%%%%%%
% % 2 inch (50.8mm) bump case
% resolution = 25;%%
% numOfOutputs = 6;
% MAX_BUMP_DISPLACMENT = 2; %[in]
% wheelDisplacement =  MAX_BUMP_DISPLACMENT*25.4; %[mm]
% wheelDisps = linspace(0,wheelDisplacement, resolution);
% dh = wheelDisplacement/resolution;
% F_mags = zeros(numOfOutputs,resolution);
% for idx = 1:resolution
%     %pause(0.01)
% 
%     p_TP = p_TP + [0,dh,0];
%     p_WC = p_WC + [0,dh,0];
% 
%     pU_UCA = rotatePoint(pU_UCA, pC_UCA_in, dh);
%     pU_LCA = rotatePoint(pU_LCA, pC_LCA_in, dh);
%     pUCA_PR = rotatePoint(pUCA_PR, pR_C, dh);
%     pR_PR = rotatePoint(pR_PR, pR_C, dh);
%     pR_S = rotatePoint(pR_S, pR_C, dh);
%     pTR_out = rotatePoint(pTR_out, pTR_in, dh);
%     %implement projection to find piot point of other points
%     %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 
% %% Computes Front Suspension Forces and Plots
% 
% %%% COMPUTE FORCES HERE AND RECORD
% 
%     % Recording Forces
%     F_mags(:,idx) = F_mag;

    %Setting up figure
    figure(1)
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Solar VDX Suspension');
    subtitle(sprintf('Front %s Side(s)', side))
    axis equal
    hold on;
    %legend show;
    grid on;

    %% Plotting
    % Drawing
    %pause(0.1)
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
    plot3(rockerPoints(:,1), rockerPoints(:,2), rockerPoints(:,3), 'DisplayName', 'Rocker')

    hold off;



for j = 1:numOfOutputs
    plot(wheelDisps/25.4,F_mags(j,:)/1000, 'DisplayName', forceNames{j})
    if j == 1
        hold on
        legend show;
        xlabel("Wheel Displacement [in]")
        ylabel("Signed Force Magnitude [kN]")
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%

%% FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%

function displayForceVectors(struct)
    fields = fieldnames(struct);
    n = numel(fields);
    forceVectors = zeros(n, 3);
    for i = 1:n
        field = fields{i};
        value = struct.(field);
        forceVectors(i, :) = value;
    end
    % Create table
    disp(array2table(forceVectors, 'VariableNames', {'X', 'Y', 'Z'}, 'RowNames', fields));
end

function displayVectorComponents(vectorNames)
    n = numel(vectorNames);
    components = zeros(n, 3);  % Preallocate matrix
    
    for i = 1:n
        forceVector(i, :) = evalin('base', vectorNames{i});
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

%% Rotate a point about a an axis parallel to X axis
function P_rotated = rotatePoint(point, axisPoint, displacement)

    % Shift Point to Origin
    P_shifted = point - axisPoint;

    % Rotation matrix about X axis --FIX! cooked cooked 
    t = asin(displacement/norm(point-axisPoint));
    %displacement/norm(point-axisPoint)
    Rx = [ 1,       0,        0;
           0,     cos(t)   sin(t);
           0,      -sin(t)   cos(t)];

    % Rotate shifted point about origin
    P_rotated_shifted = Rx * P_shifted';

    P_rotated = P_rotated_shifted' + axisPoint; 
end






