%%%%%% Script to Implement Suspension Travel %%%%%%
clear; close; clc
run("getCoordinatesV3.m")

%% Calculate Tire Contact Patch Location
tireRadius = 283; %mm

%% Setup Plot

clc; clf
figure(1)
v1=[0,0,0]; v2=[0,2,2];
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Suspension Hardpoints');
% subtitle('Driver Left Side?')
hold on;
legend show;
legend('location', 'westoutside')
view(3); % Set 3D view
xlim([-500,400])
zlim([-300,400])
set(gcf, 'Color', 'w');

% Direction Vectors
u_tieRod = p_tr - p_tk;
u_spring = p_st - p_sb;
u_LCA_F = p_bf_low - p_ball_low;
u_LCA_R = p_br_low - p_ball_low;
u_UCA_F = p_bf_up - p_ball_up;
u_UCA_R = p_br_up - p_ball_up;


%% Bumping
bump_range = linspace(10, -11.111111111, 20);   % +/- 10 mm? bump - units are not accurate

% Assume MNQ is flat and orthogonal to the front of the car
% Assume rotation axis is paralell to y-axis

rotationPoint_LCA = [p_bf_low(1), p_ball_low(2), p_bf_low(3)];
rotationPoint_UCA =  [p_bf_up(1), p_ball_up(2), p_bf_up(3)];
rotationPoint_tieRod = [p_tr(1), p_tk(2), p_tr(3)];
rotationPoint_spring = [p_st(1), p_sb(2), p_st(3)];

% GIF File Name
filename = 'susTravel6.gif';  

for i = 1:length(bump_range)

    %axis manual;
    cla; % Only clear axes, not figure
    
    bump = bump_range(i);
    y = bump;

    if i == 1
        wc_initial = p_wc(3);
    end

    subtitle(sprintf('Bump Travel: %02.f mm',  p_wc(3)- wc_initial))

    % Recompute Direction Vectors
    u_tieRod = p_tr - p_tk;
    u_spring = p_st - p_sb;
    u_LCA_F = p_bf_low - p_ball_low;
    u_LCA_R = p_br_low - p_ball_low;
    u_UCA_F = p_bf_up - p_ball_up;
    u_UCA_R = p_br_up - p_ball_up;

    % Tire bummping
    p_wc = p_wc + bump;
    p_tp = [p_wc(1), p_wc(2), p_wc(3)-tireRadius];

    % Sus members bumping
    p_ball_low = rotatePoint(p_ball_low, rotationPoint_LCA,bump);
    p_ball_up = rotatePoint(p_ball_up, rotationPoint_UCA,bump);
    p_tk = rotatePoint(p_tk, rotationPoint_tieRod, bump);
    p_sb = rotatePoint(p_sb, rotationPoint_spring, bump);

%     %Plot Rotation stuff
%     drawPoint(rotationPoint_UCA, 'rotationPoint_{UCA}', 'black')
%     drawPoint(rotationPoint_LCA, 'rotationPoint_{LCA}', 'black')
%     drawPoint(rotationPoint_tieRod, 'rotationPoint_{tieRod}', 'black')
%     drawPoint(rotationPoint_spring, 'rotationPoint_{spring}', 'black')

    % Plot sus links
    drawLink(p_tk, u_tieRod, "Tie Rod", "r")
    drawLink(p_ball_low, u_LCA_F, "LCA_F", 'b')
    drawLink(p_ball_low, u_LCA_R, "LCA_R", 'g')
    drawLink(p_ball_up, u_UCA_F, "UCA_F", 'm')
    drawLink(p_ball_up, u_UCA_R, "UCA_R", 'k')
    drawLink(p_sb, u_spring, "Shock", 'cyan')
    
    % Plot wheel center and tire patch
    scatter3(p_wc(1), p_wc(2), p_wc(3), 50, 'b', 'filled', 'DisplayName','Wheel Center')
    scatter3(p_tp(1), p_tp(2), p_tp(3), 50, 'r', 'filled', 'DisplayName','Tire Patch')
    drawCircle(p_wc, [0,1,0], tireRadius, "Tire", "k")

    drawnow;

    % Capture the plot as an image
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);

    % Write to the GIF File
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end
%% Rotate a point about a an axis parallel to Y axis
function P_rotated = rotatePoint(point, axisPoint, displacement)

    % Shift Point to Origin
    P_shifted = point - axisPoint;

    % Rotation matrix about Y axis
    t = asin(displacement/norm(point-axisPoint));
    Ry = [cos(t),0,-sin(t);
                           0, 1, 0;
                      sin(t), 0, cos(t)];

    % Rotate shifted point about origin
    P_rotated_shifted = Ry * P_shifted';

    P_rotated = P_rotated_shifted' + axisPoint; 
end
%% Draw point
function drawPoint(coordinates, name, color)
    scatter3(coordinates(1),coordinates(2),coordinates(3), 'DisplayName', name, 'MarkerFaceColor',color, 'MarkerEdgeColor',color)
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







