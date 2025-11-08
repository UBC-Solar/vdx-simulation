function plotLinks(p)
    drawLink(p.TR_out, p.TR_in - p.TR_out, "TR", "r")
    drawLink(p.U_LCA, p.C_LCA_in - p.U_LCA, "LCA_{in}", 'b')
    drawLink(p.U_LCA, p.C_LCA_out - p.U_LCA, "LCA_{out}", 'g')
    drawLink(p.U_UCA, p.C_UCA_in - p.U_UCA, "UCA_{in}", 'm')
    drawLink(p.U_UCA, p.C_UCA_out - p.U_UCA, "UCA_{out}", 'k')
    drawLink(p.UCA_PR, p.R_PR - p.UCA_PR, "PR", 'cyan')

    % p.lot wheel center and tire p.atch
    tireRadius = norm(p.WC-p.TP);
    scatter3(p.WC(1), p.WC(2), p.WC(3), 50, 'b', 'filled', 'DisplayName','WC')
    scatter3(p.TP(1), p.TP(2), p.TP(3), 50, 'r', 'filled', 'DisplayName','TP')
    drawCircle(p.WC, [1,0,0], tireRadius, "Tire", "k")

    % Rocker p.lotting
    drawPoint(p.R_C, 'p.R_C', 'k')
    drawPoint(p.R_PR, 'p.R_{p.R}', 'o')
    drawPoint(p.R_S, 'p.R_S', 'g')
    rockerpoints = [p.R_C; p.R_PR; p.R_S; p.R_C];

    % Connect rocker p.oints with line
    plot3(rockerpoints(:,1), rockerpoints(:,2), rockerpoints(:,3), 'DisplayName', 'Rocker')
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
