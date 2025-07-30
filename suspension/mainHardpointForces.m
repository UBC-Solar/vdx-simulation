% Calcualting Hardpoint forces Main script %
clear, close, clc


%% Getting coordinates from google sheet
run("getCoordinatesV3.m")

%% Set Loading Condition
%regs: 1g turn, 2g bump, 1g braking
bumpG = 2 %bump should >= 1 (no bump would be bumpG = 1 for static weight)
brakeG = 1
cornerG = 1
turnDirection = 1;
% 1 right, -1 left


%% Get Calculate Forces at Tire Patch
run("tirePatchForces.m");

%% Direction Vectors
u_tieRod = pTR_in - pTR_out;
u_LCA_in = pC_LCA_in - pU_LCA; %inboard was front and outboard was rear
u_LCA_out = pC_LCA_out - pU_LCA; 
u_UCA_in = pC_UCA_in - pU_UCA;
u_UCA_out = pC_UCA_out - pU_UCA;
u_PR = pR_PR - pUCA_PR;

%% Plotting
clc; clf
figure(1)
v1=[0,0,0]; v2=[0,2,2];
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Solar VDX Suspension');
%subtitle('Driver Left Side')
hold on;

drawLink(pTR_out, u_tieRod, "TR", "r")
drawLink(pU_LCA, u_LCA_in, "LCA_{in}", 'b')
drawLink(pU_LCA, u_LCA_out, "LCA_{out}", 'g')
drawLink(pU_UCA, u_UCA_in, "UCA_{in}", 'm')
drawLink(pU_UCA, u_UCA_out, "UCA_{out}", 'k')
drawLink(pUCA_PR, u_PR, "PR", 'cyan')

% Plot wheel center and tire patch
tireRadius = norm(p_WC-p_TP);
scatter3(p_WC(1), p_WC(2), p_WC(3), 50, 'b', 'filled', 'DisplayName','WC')
scatter3(p_TP(1), p_TP(2), p_TP(3), 50, 'r', 'filled', 'DisplayName','TP')
drawCircle(p_WC, [0,1,0], tireRadius, "Tire", "k")

legend show;
view(3); % Set 3D view

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
d_tr = (pTR_in - p_O);
d_bf_low = (pC_LCA_in - p_O);
d_br_low = (pC_LCA_out - p_O);
d_bf_up = (pC_UCA_in - p_O);
d_br_up = (pC_UCA_out - p_O);
d_st = (pR_PR - p_O);


%% Calculate momemnt unit vectors for each chassis point 
% using the moment arms and force unit vectors
uM_tr = cross(d_tr, u_tieRod);
uM_bf_low = cross(d_bf_low, u_LCA_in);
uM_br_low = cross(d_br_low, u_LCA_out);
uM_bf_up = cross(d_br_up, u_UCA_in);
uM_br_up = cross(d_br_up, u_UCA_out);
uM_st = cross(d_st, u_PR);

%% Calcualte moment due to input force at tire contact patch
d_TP = (p_TP - MC)/1000;
M_TP = cross(d_TP, f_FL_TP);

%% Solve the system (Ax = b)

A = [u_tieRod', u_LCA_in', u_LCA_out', u_UCA_in', u_UCA_out', u_PR';
    uM_tr'/1000, uM_bf_low'/1000, uM_bf_low'/1000, uM_br_low'/1000, uM_br_up'/1000, uM_st'/1000];

b = [-f_FL_TP';
    -M_TP'];

format short
F_mag = A\b;

disp('Forces displyed in Newtons [x y z]')

%% Scale Unit Vectors by force magnitude to obtain force vectors
F_tieRod = F_mag(1) * u_tieRod
F_LCA_in = F_mag(2) * u_LCA_in
F_LCA_out = F_mag(3) * u_LCA_out
F_UCA_in = F_mag(4) * u_UCA_in
F_UCA_out = F_mag(5) * u_UCA_out
F_PR = F_mag(6) * u_PR

%% Running Rocker Script
run('rockerForcesV1.m')

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
    'F_tieRod';
    'F_LCA_in';
    'F_LCA_out';
    'F_UCA_in';
    'F_UCA_out';
    'F_PR';
    'F_RC';
    'F_S'
};

copyForceTableForGoogleDocs(forceNames, forces);

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


function T = makeForceTable(forceNames, forces)
% makeForceTable - Create a table with force names and 3D components
%
% Syntax:
%   T = makeForceTable(forceNames, forces)
%
% Inputs:
%   forceNames - Cell array of strings, e.g. {'F_tieRod', 'F_LCA_in'}
%   forces     - Nx3 matrix of [X Y Z] force components
%
% Output:
%   T - MATLAB table with columns: Force Name, X (N), Y (N), Z (N)

    if length(forceNames) ~= size(forces, 1)
        error('Number of names must match number of force vectors.');
    end

    % Create table
    T = table(forceNames(:), forces(:,1), forces(:,2), forces(:,3), ...
        'VariableNames', {'ForceName', 'X_N', 'Y_N', 'Z_N'});
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
    axis equal
    grid on
end











