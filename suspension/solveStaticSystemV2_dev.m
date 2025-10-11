%% Computes Front Suspension Forces and Plots

% Determining if we are computing the front left or right 
if p_WC(2) > 0
    f_TP = f_FR_TP;
else
    f_TP = f_FL_TP;
end



%% Plotting

% drawLink(pTR_out, u_tieRod, "TR", "r")
% drawLink(pU_LCA, u_LCA_in, "LCA_{in}", 'b')
% drawLink(pU_LCA, u_LCA_out, "LCA_{out}", 'g')
% drawLink(pU_UCA, u_UCA_in, "UCA_{in}", 'm')
% drawLink(pU_UCA, u_UCA_out, "UCA_{out}", 'k')
% drawLink(pUCA_PR, u_PR, "PR", 'cyan')
% 
% % Plot wheel center and tire patch
% tireRadius = norm(p_WC-p_TP);
% scatter3(p_WC(1), p_WC(2), p_WC(3), 50, 'b', 'filled', 'DisplayName','WC')
% scatter3(p_TP(1), p_TP(2), p_TP(3), 50, 'r', 'filled', 'DisplayName','TP')
% drawCircle(p_WC, [1,0,0], tireRadius, "Tire", "k")

%% Known Direction Vectors
u_tieRod = pTR_in - pTR_out; %inboard was front and outboard was rear
% u_LCA_in = pC_LCA_in - pU_LCA; 
% u_LCA_out = pC_LCA_out - pU_LCA; 
u_UCA_in = pC_UCA_in - pU_UCA;
u_UCA_out = pC_UCA_out - pU_UCA;
u_PR = pR_PR - pUCA_PR;

% Normalizing;
u_tieRod = u_tieRod/norm(u_tieRod);
% u_LCA_in = u_LCA_in/norm(u_LCA_in);
% u_LCA_out = u_LCA_out/norm(u_LCA_out);
u_UCA_in = u_UCA_in/norm(u_UCA_in);
u_UCA_out = u_UCA_out/norm(u_UCA_out);
u_PR = u_PR/norm(u_PR);

%% Calculate Moment Arms from each chassis point
MC = p_O; % moment center setting as origin for now
%WILL NEED TO BE A DATA FIELD LATER FOR CORNER (SINCE DIFFERNET FOR FRONT AND BACK)
d_tieRod = (pTR_in - p_O);
% d_LCA_in = (pC_LCA_in - p_O);
% d_LCA_out = (pC_LCA_out - p_O);
d_UCA_in = (pC_UCA_in - p_O);
d_UCA_out = (pC_UCA_out - p_O);
d_PR = (pR_PR - p_O);


%% Calculate momemnt unit vectors for each chassis point 
% using the moment arms and force unit vectors
uM_tieRod = cross(d_tieRod, u_tieRod);
% uM_LCA_in = cross(d_LCA_in, u_LCA_in);
% uM_LCA_out = cross(d_LCA_out, u_LCA_out);
uM_UCA_in = cross(d_UCA_out, u_UCA_in);
uM_UCA_out = cross(d_UCA_out, u_UCA_out);
uM_PR = cross(d_PR, u_PR);

%% Calcualte moment due to input force at tire contact patch
d_TP = (p_TP - MC);
M_TP = cross(d_TP, f_TP);

%% Solve force system

function F_mag = solveForceMagnitudes(u_tieRod, u_UCA_in, u_UCA_out, u_PR, uM_tieRod, uM_UCA_in, uM_UCA_out, uM_PR, t)

% R_C = lamA uA + lamB uB

A = [u_tieRod', u_LCA_in', u_LCA_out', u_UCA_in', u_UCA_out', u_PR';
    uM_tieRod', uM_LCA_in', uM_LCA_out', uM_UCA_in', uM_UCA_out', uM_PR'];

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











