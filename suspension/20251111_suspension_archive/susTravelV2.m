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
%side = "Right"; % (coordiante driving primary side)
side = "Left";
%side = "Both";

%%%%%% Vehicle Paramters %%%%%%
totalMass = 354.37; %kg
trackWidth = 1270; %mm
wheelBase = 2750; %mm
g = 9.81; % gravatiational acceleration

%%%%%% Set Plot View %%%%%%
sw_view('right');

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

% BUMPING %%%%%%%%%%%%%%%%%%%%
% 2 inch (50.8mm) bump case
resolution = 25;%%
numOfOutputs = 6;
MAX_BUMP_DISPLACMENT = 2; %[in]
wheelDisplacement =  MAX_BUMP_DISPLACMENT*25.4; %[mm]
wheelDisps = linspace(0,wheelDisplacement, resolution);
dh = wheelDisplacement/resolution;
F_mags = zeros(numOfOutputs,resolution);
for idx = 1:resolution
    %pause(0.01)
   
    p_TP = p_TP + [0,dh,0];
    p_WC = p_WC + [0,dh,0];
    
    pU_UCA = rotatePoint(pU_UCA, pC_UCA_in, dh);
    pU_LCA = rotatePoint(pU_LCA, pC_LCA_in, dh);
    pUCA_PR = rotatePoint(pUCA_PR, pR_C, dh);
    pR_PR = rotatePoint(pR_PR, pR_C, dh);
    pR_S = rotatePoint(pR_S, pR_C, dh);
    pTR_out = rotatePoint(pTR_out, pTR_in, dh);
    %implement projection to find piot point of other points
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
%% Computes Front Suspension Forces and Plots

% Determining if we are computing the front left or right 
if p_WC(1) < 0
    f_TP = f_FR_TP;
else
    f_TP = f_FL_TP;
end


% 2 inch (50.8mm) bump case
% MAX_BUMP_DISPLACMENT = 2; %[in]
% wheelDisplacement = MAX_BUMP_DISPLACMENT*25.4; %[mm]
% 
% pU_UCA = rotatePoint(pU_UCA, pC_UCA_in, wheelDisplacement);
% pU_LCA = rotatePoint(pU_LCA, pC_LCA_in, wheelDisplacement);
% pUCA_PR = rotatePoint(pUCA_PR, pR_C, wheelDisplacement);
% pR_PR = rotatePoint(pR_PR, pR_C, wheelDisplacement);
% pR_S = rotatePoint(pR_S, pR_C, wheelDisplacement);
%pTR_out = rotatePoint


%ZZZ TESTING
% XXX = norm(pU_UCA - pC_UCA_in);
% p_0 = pU_UCA;
% pU_UCA = rotatePoint(pU_UCA, pC_UCA_in, 3);
% YYY = norm(p_0-pU_UCA);
% ANGLE = atand(YYY/XXX)
% ANGLE2 = atand(3/XXX)

%% Direction Vectors
u_tieRod = pTR_in - pTR_out;
u_LCA_in = pC_LCA_in - pU_LCA; %inboard was front and outboard was rear
u_LCA_out = pC_LCA_out - pU_LCA; 
u_UCA_in = pC_UCA_in - pU_UCA;
u_UCA_out = pC_UCA_out - pU_UCA;
u_PR = pR_PR - pUCA_PR;

% Normalizing;
u_tieRod = u_tieRod/norm(u_tieRod);
u_LCA_in = u_LCA_in/norm(u_LCA_in);
u_LCA_out = u_LCA_out/norm(u_LCA_out);
u_UCA_in = u_UCA_in/norm(u_UCA_in);
u_UCA_out = u_UCA_out/norm(u_UCA_out);
u_PR = u_PR/norm(u_PR);

%% Calculate Moment Arms from each chassis point
MC = p_O; % moment center setting as origin for now
%WILL NEED TO BE A DATA FIELD LATER FOR CORNER (SINCE DIFFERNET FOR FRONT AND BACK)
d_tieRod = (pTR_in - p_O);
d_LCA_in = (pC_LCA_in - p_O);
d_LCA_out = (pC_LCA_out - p_O);
d_UCA_in = (pC_UCA_in - p_O);
d_UCA_out = (pC_UCA_out - p_O);
d_PR = (pR_PR - p_O);


%% Calculate momemnt unit vectors for each chassis point 
% using the moment arms and force unit vectors
uM_tieRod = cross(d_tieRod, u_tieRod);
uM_LCA_in = cross(d_LCA_in, u_LCA_in);
uM_LCA_out = cross(d_LCA_out, u_LCA_out);
uM_UCA_in = cross(d_UCA_out, u_UCA_in);
uM_UCA_out = cross(d_UCA_out, u_UCA_out);
uM_PR = cross(d_PR, u_PR);

%% Calcualte moment due to input force at tire contact patch
d_TP = (p_TP - MC);
M_TP = cross(d_TP, f_TP);

%% Solve the system (Ax = b)

A = [u_tieRod', u_LCA_in', u_LCA_out', u_UCA_in', u_UCA_out', u_PR';
    uM_tieRod', uM_LCA_in', uM_LCA_in', uM_UCA_in', uM_UCA_out', uM_PR'];

b = [-f_TP';
    -M_TP'];

F_mag = A\b;

%% Scale Unit Vectors by force magnitude to obtain force vectors
F_tieRod = F_mag(1) * u_tieRod;
F_LCA_in = F_mag(2) * u_LCA_in;
F_LCA_out = F_mag(3) * u_LCA_out;
F_UCA_in = F_mag(4) * u_UCA_in;
F_UCA_out = F_mag(5) * u_UCA_out;
F_PR = F_mag(6) * u_PR;

%zzz_F_sum = F_tieRod+F_LCA_in+F_LCA_out+F_UCA_in+F_UCA_out+F_PR

%% Running Rocker Routine
% Rocker Force Analysis
% Run mainHardpointForces.m! this script should not be run on own
% Assumming rocker plane is parallel to the YZ plane (x constant)

% Pull Rod unit vector
u_PR = F_PR/norm(F_PR);

% Shock unit direction vector
u_S = (pC_S - pR_C)/norm(pC_S - pR_C);

 % Moment about rocker chassis point
 dR_PR = pR_PR - pR_C;
 dR_S = pR_S - pR_C;

 % Unit moment vectors
 uMR_PR = cross(dR_PR, u_PR);
 uMR_S = cross(dR_S, u_S);

 % Linear System
 AR = [1, 0, u_S(y);
      0, 1, u_S(z);
      zeros(3,1), zeros(3,1), uMR_S'];

 bR = [-F_PR(y);
     -F_PR(z);
     -(norm(F_PR)*uMR_PR)'];

FR = AR\bR;

% Rocker-Chassis Force, Rocker Shock Force
F_RC = [0, FR(1), FR(2)];
F_S = FR(3)*u_S; 



%% Put Forces in a table
forces = [
    F_tieRod;
    F_LCA_in;
    F_LCA_out;
    F_UCA_in;
    F_UCA_out;
    F_PR;
    F_RC;
    F_S];

forceNames = {
    'F_{tieRod}';
    'F_{LCA_{in}}';
    'F_{LCA_{out}}';
    'F_{UCA_{in}}';
    'F_{UCA_{out}}';
    'F_{PR}';
    'F_{RC}';
    'F_{S}'
};

    % Recording Forces
    F_mags(:,idx) = F_mag;

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
    plot3(rockerPoints(:,x), rockerPoints(:,y), rockerPoints(:,z), 'DisplayName', 'Rocker')

    hold off;
end



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






