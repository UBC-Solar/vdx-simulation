% Author: Graeme Dockrill, 2022
% Modified by Liam Foster, 2023
% This script takes an excel file 'setup.xlsx' and plots the suspension
% system in 3D and calculates the forces in each member

% Clear the console and all variables
clear
clc

% ----------------Setup----------------------------------------------------

% Constants
hardpointsize = 25;
LineWidth = 1.5;
unitvect_scalefactor = 0.05;
unitvect_linewidth = 1.5;
forcevect_scalefactor = 0.00005;
forcevect_linewidth = 1.5;

% Carbon Tube Cross-Sectionial Area
innerDiameter_inch = 0.75;
outerDiameter_inch = 0.86;
%ultimateStrengthPa = 3.5*10^9;
%ultimateStrengthPa = 240*10^6;

utimateStrength_psi = 125000;
ultimateStrengthPa = utimateStrength_psi*6894.76;

innerRadius_inch = innerDiameter_inch/2;
outerRadius_inch = outerDiameter_inch/2;
innerRadius_meter = innerRadius_inch*0.0254;
outerRadius_meter = outerRadius_inch*0.0254;
crossSectionArea = pi*(outerRadius_meter^2 - innerRadius_meter^2);

% innerDiameter_mm = 3;
% outerDiameter_mm = 5;
% %ultimateStrengthPa = 3.5*10^9;
% ultimateStrengthPa = 240*10^6;
% 
% innerRadius_meter = innerDiameter_mm/1000/2;
% outerRadius_meter = outerDiameter_mm/1000/2;
% crossSectionArea = pi*(outerRadius_meter^2 - innerRadius_meter^2);

% x,y,z for clearer equations
x = 1;
y = 2;
z = 3;

% Extracts balljoint coordinates from VideoSuspensionForcesSetup.xlsx
exceltable = readtable('VideoSuspensionForcesSetup','NumHeaderLines',19);
coordinates = table2array(exceltable);

% Converting values read from excel to 3D points
O = coordinates(x:z,1);
I_LCA_R = coordinates(x:z,2);
O_LCA = coordinates(x:z,3);
I_LCA_F= coordinates(x:z,4);
I_UCA_R = coordinates(x:z,5);
O_UCA = coordinates(x:z,6);
I_UCA_F = coordinates(x:z,7);
O_PR = coordinates(x:z,8);
I_PR = coordinates(x:z,9);
O_TR = coordinates(x:z,10);
I_TR = coordinates(x:z,11);
WC = coordinates(x:z,12);
TP = coordinates(x:z,13);
MomCent = coordinates(x:z,14);

% Converting input forces from excel to 3D vectors
F = coordinates(6:8,1);


% Setting labels for each point in coordinates
labels = {'Origin', 'I_LCA_R', 'O_LCA', 'I_LCA_F', 'I_UCA_R', 'O_UCA', 'I_UCA_F', 'O_PR', 'I_PR', 'O_TR', 'I_TR', 'WC', 'TP'};


% ----------------Plotting-------------------------------------------------


% Plotting all points for control arms
scatter3(coordinates(x,:), coordinates(y,:), coordinates(z,:), hardpointsize, 'filled', 'o', 'blue');

hold on


% Plotting the contact patch force
% quiver3(TP(x), TP(y), TP(z), F(x), F(y), F(z), 'Color', 'cyan', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);

% Plotting x,y,z components of contact patch force
quiver3(TP(x), TP(y), TP(z), F(x), 0, 0, 'Color', 'cyan', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
quiver3(TP(x), TP(y), TP(z), 0, F(y), 0, 'Color', 'cyan', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
quiver3(TP(x), TP(y), TP(z), 0, 0, F(z), 'Color', 'cyan', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);


% Drawing lines for UCA, LCA, Shock, and Tie rod
line(coordinates(x,2:4), coordinates(y,2:4), coordinates(z,2:4),'Color','blue', 'LineWidth', LineWidth);
line(coordinates(x,5:7), coordinates(y,5:7), coordinates(z,5:7),'Color','blue', 'LineWidth', LineWidth);
line(coordinates(x,8:9), coordinates(y,8:9), coordinates(z,8:9),'Color','black', 'LineWidth', LineWidth);
line(coordinates(x,10:11), coordinates(y,10:11), coordinates(z,10:11),'Color','black', 'LineWidth', LineWidth);

% Labelling axes and setting their aspect ratio
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m');
axis equal

% Labelling each point in the plot
text(coordinates(x,1:13), coordinates(y,1:13), coordinates(z,1:13), labels, 'HorizontalAlignment','center', 'VerticalAlignment','bottom');
% Labelling Moment Center separately
text(coordinates(x,14), coordinates(y,14), coordinates(z,14), 'MomCent', 'HorizontalAlignment','center', 'VerticalAlignment','top');


% Drawing tire
t = linspace(0,2*pi);
tx = WC(3)*sin(t) + WC(1);
ty = 0*t + WC(2);
tz = WC(3)*cos(t) + WC(3);
pnts = [tx;ty;tz];
plot3(pnts(1,:),pnts(2,:),pnts(3,:), 'LineWidth', LineWidth)


% ----------------Calculating unit vector of members-----------------------


% getting unit vectors along each member
% (inboard - outboard coordinates) / magnitude
uLCA_F = (O_LCA-I_LCA_F)/(norm(O_LCA-I_LCA_F)); % the same as "n" shown in the matrix A
uLCA_R = (O_LCA-I_LCA_R)/(norm(O_LCA-I_LCA_R));
uUCA_F = (O_UCA-I_UCA_F)/(norm(O_UCA-I_UCA_F));
uUCA_R = (O_UCA-I_UCA_R)/(norm(O_UCA-I_UCA_R));
uPR = (O_PR-I_PR)/(norm(O_PR-I_PR));
uTR = (O_TR-I_TR)/(norm(O_TR-I_TR));

% plotting unit vector for bottom front member
quiver3(I_LCA_F(x), I_LCA_F(y), I_LCA_F(z), uLCA_F(x), uLCA_F(y), uLCA_F(z), 'color', 'green', 'AutoScaleFactor', unitvect_scalefactor, 'LineWidth', unitvect_linewidth);
% plotting unit vector for bottom rear member
quiver3(I_LCA_R(x), I_LCA_R(y), I_LCA_R(z), uLCA_R(x), uLCA_R(y), uLCA_R(z), 'color', 'green', 'AutoScaleFactor', unitvect_scalefactor, 'LineWidth', unitvect_linewidth);
% plotting unit vector for top front member
quiver3(I_UCA_F(x), I_UCA_F(y), I_UCA_F(z), uUCA_F(x), uUCA_F(y), uUCA_F(z), 'color', 'green', 'AutoScaleFactor', unitvect_scalefactor, 'LineWidth', unitvect_linewidth);
% plotting unit vector for top rear member
quiver3(I_UCA_R(x), I_UCA_R(y), I_UCA_R(z), uUCA_R(x), uUCA_R(y), uUCA_R(z), 'color', 'green', 'AutoScaleFactor', unitvect_scalefactor, 'LineWidth', unitvect_linewidth);
% plotting unit vector for shock member
quiver3(I_PR(x), I_PR(y), I_PR(z), uPR(x), uPR(y), uPR(z), 'color', 'green', 'AutoScaleFactor', unitvect_scalefactor, 'LineWidth', unitvect_linewidth);
% plotting unit vector for tie rod
quiver3(I_TR(x), I_TR(y), I_TR(z), uTR(x), uTR(y), uTR(z), 'color', 'green', 'AutoScaleFactor', unitvect_scalefactor, 'LineWidth', unitvect_linewidth);


% ----------------Calculating Moments--------------------------------------


% getting moment arm from MomCent for each member
rTR = I_TR-MomCent;
rPR = I_PR-MomCent;
rLCA_R = I_LCA_R-MomCent;
rLCA_F = I_LCA_F-MomCent;
rUCA_R = I_UCA_R-MomCent;
rUCA_F = I_UCA_F-MomCent;
rTP = TP-MomCent;

% Mx = rxF

% Moments around MomCent from input forces
M_Fx = cross(rTP,[F(x); 0; 0])
M_Fy = cross(rTP,[0; F(y); 0])
M_Fz = cross(rTP,[0; 0; F(z)])


% Moments for each member about the origin
M_UCA_F = cross(rUCA_F, uUCA_F);
M_UCA_R = cross(rUCA_R, uUCA_R);
M_LCA_F = cross(rLCA_F, uLCA_F);
M_LCA_R = cross(rLCA_R, uLCA_R);
M_PR = cross(rPR, uPR);
M_TR = cross(rTR, uTR);


% ----------------Setting up Matrix to solve-------------------------------


% 6x6 matrix from sum of forces and sum of moments about MomCent
A = [uUCA_F(x),    uUCA_R(x),     uLCA_F(x),     uLCA_R(x),     uPR(x),     uTR(x);
     uUCA_F(y),    uUCA_R(y),     uLCA_F(y),     uLCA_R(y),     uPR(y),     uTR(y);
     uUCA_F(z),    uUCA_R(z),     uLCA_F(z),     uLCA_R(z),     uPR(z),     uTR(z);
     M_UCA_F(x),   M_UCA_R(x),    M_LCA_F(x),    M_LCA_R(x),    M_PR(x),    M_TR(x);
     M_UCA_F(y),   M_UCA_R(y),    M_LCA_F(y),    M_LCA_R(y),    M_PR(y),    M_TR(y);
     M_UCA_F(z),   M_UCA_R(z),    M_LCA_F(z),    M_LCA_R(z),    M_PR(z),    M_TR(z)];


F; % imported from spreadsheet

% Make input matrix B=[-F(x); -F(y); -F(z); -M(x); -M(y); -M(z)]
B = [-F(x); -F(y); -F(z); -(M_Fx(x)+M_Fy(x)+M_Fz(x)); -(M_Fx(y)+M_Fy(y)+M_Fz(y)); -(M_Fx(z)+M_Fy(z)+M_Fz(z))];


% ----------------Solve for Forces in Members------------------------------


X = inv(A)*B

% Formatting of X: X = [F_UCA_F, F_UCA_R, F_LCA_F, F_LCA_R, F_PR, F_TR]

% Each member force as a 3x1 vector
F_UCA_F = X(1)*uUCA_F;
F_UCA_R = X(2)*uUCA_R;
F_LCA_F = X(3)*uLCA_F;
F_LCA_R = X(4)*uLCA_R;
F_PR = X(5)*uPR;
F_TR = X(6)*uTR;
F_Mat = [F_UCA_F F_UCA_R F_LCA_F F_LCA_R F_PR F_TR]';

% ----------------Plotting Force vectors for Members-----------------------


% plotting Force vector along bottom front member
quiver3(I_LCA_F(x), I_LCA_F(y), I_LCA_F(z), F_LCA_F(x), F_LCA_F(y), F_LCA_F(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% plotting Force vector along bottom rear member
quiver3(I_LCA_R(x), I_LCA_R(y), I_LCA_R(z), F_LCA_R(x), F_LCA_R(y), F_LCA_R(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% plotting Force vector along top front member
quiver3(I_UCA_F(x), I_UCA_F(y), I_UCA_F(z), F_UCA_F(x), F_UCA_F(y), F_UCA_F(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% plotting Force vector along top rear member
quiver3(I_UCA_R(x), I_UCA_R(y), I_UCA_R(z), F_UCA_R(x), F_UCA_R(y), F_UCA_R(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% plotting Force vector along shock member
quiver3(I_PR(x), I_PR(y), I_PR(z), F_PR(x), F_PR(y), F_PR(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% plotting Force vector along tie rod
quiver3(I_TR(x), I_TR(y), I_TR(z), F_TR(x), F_TR(y), F_TR(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);


% % plotting Force vector components for bottom front member
% quiver3(I_LCA_F(x), I_LCA_F(y), I_LCA_F(z), F_LCA_F(x), 0, 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_LCA_F(x), I_LCA_F(y), I_LCA_F(z), 0, F_LCA_F(y), 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_LCA_F(x), I_LCA_F(y), I_LCA_F(z), 0, 0, F_LCA_F(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% % plotting Force vector components for bottom rear member
% quiver3(I_LCA_R(x), I_LCA_R(y), I_LCA_R(z), F_LCA_R(x), 0, 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_LCA_R(x), I_LCA_R(y), I_LCA_R(z), F_LCA_R(y), 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_LCA_R(x), I_LCA_R(y), I_LCA_R(z), 0, 0, F_LCA_R(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% % plotting Force vector components for top front member
% quiver3(I_UCA_F(x), I_UCA_F(y), I_UCA_F(z), F_UCA_F(x), 0, 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_UCA_F(x), I_UCA_F(y), I_UCA_F(z), 0, F_UCA_F(y), 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_UCA_F(x), I_UCA_F(y), I_UCA_F(z), 0, 0, F_UCA_F(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% % plotting Force vector fcomponents or top rear member
% quiver3(I_UCA_R(x), I_UCA_R(y), I_UCA_R(z), F_UCA_R(x), 0, 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_UCA_R(x), I_UCA_R(y), I_UCA_R(z), 0, F_UCA_R(y), 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_UCA_R(x), I_UCA_R(y), I_UCA_R(z), F_UCA_R(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% % plotting Force vector components for shock member
% quiver3(I_PR(x), I_PR(y), I_PR(z), F_PR(x), 0, 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_PR(x), I_PR(y), I_PR(z), 0, F_PR(y), 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_PR(x), I_PR(y), I_PR(z), 0, 0, F_PR(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% % plotting Force vector components for tie rod
% quiver3(I_TR(x), I_TR(y), I_TR(z), F_TR(x), 0, 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_TR(x), I_TR(y), I_TR(z), 0, F_TR(y), 0, 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);
% quiver3(I_TR(x), I_TR(y), I_TR(z), 0, 0, F_TR(z), 'color', 'red', 'AutoScaleFactor', forcevect_scalefactor, 'LineWidth', forcevect_linewidth);


% ----------------Computing Stresses in Members----------------------------

% computing stresses in each member
sigma = X/crossSectionArea
% Formatting of sigma: sigma = [sigma_UCA_F, sigma_UCA_R, sigma_LCA_F, sigma_LCA_R, sigma_PR, sigma_TR]


% safety factors of each member
FoS = abs(ultimateStrengthPa./sigma)
% Formatting of FoS: FoS = [FoS_UCA_F, FoS_UCA_R, FoS_LCA_F, FoS_LCA_R, FoS_PR, FoS_TR]
