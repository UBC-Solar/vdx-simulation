%% Steering Performance Analysis (Munro's Method)
% Generates performance plots for steering geometry across full travel.
%
% This script produces:
%   - Turning radius plot
%   - Induced power loss plot
%   - IC distance plot (Ackermann regions)
%   - Parasitic camber plot
%   - Induced wheel slip angle plot
%   - Dynamic toe plot
%   - Pure rolling closeness plot
%   - Wheel steer angle plot
%   - Lateral acceleration plot
%
% Run steeringGeometry.m first to initialize workspace variables.

% Initialize
clear; format shortG; close all;

% Load shared geometry configuration


SAvec = [-160,  -45];         % Steering arm offset [X-inset, Y-inset] %[control:editfield:4e3c]{"position":[9,21]}
ERconnectionLen = 30;      % Extension rod connection length %[control:editfield:209a]{"position":[19,21]}
setback = 340;              % X distance, ER axis to WC %[control:editfield:9564]{"position":[11,14]}
steeringGeometry;


% Analysis Parameters
twoSided = false; %[control:statebutton:816f]{"position":[12,17]}
n = 2^7; %[control:slider:24b5]{"position":[7,8]}

% Generate Dataframe
df = genPerformanceTable(car, sNodes, sNodesStarboard, ...
    ERaxisFun, ERaxisFunStarboard, findRackPos, maxYoke, n, twoSided);

% Reference Lines for Plots
if twoSided
    referenceYoke = [-75, 75];
else
    referenceYoke = 75;
end

% Plot 1: Turning Radius
figure('Name', 'Turning Radius', 'NumberTitle', 'off');
hold on;
plot(df.yokePos, df.radiusOuter/1e3, 'r', DisplayName='Outer Wheel');
plot(df.yokePos, df.radiusInner/1e3, 'g', DisplayName='Inner Wheel');
plot(df.yokePos, df.radiusEff/1e3, 'y', DisplayName='Effective');
yline(12, 'k--', DisplayName='Tightest Road Radius');
yline(15/2, 'c--', DisplayName='U-turn Requirement');
xline(referenceYoke, '--', HandleVisibility='off');
ylim([0 30]);
xlabel('Yoke Position [°]');
ylabel('Corner Radius (@ front wheel) [m]');
legend('Location', 'best');
title('Turning Tightness');
grid on;

% Plot 2: Induced Power Loss
figure('Name', 'Induced Power Loss', 'NumberTitle', 'off');
hold on;
plot(df.yokePos, df.inducedPowerLossTot, 'b', DisplayName='Total');
plot(df.yokePos, df.inducedPowerLossL, 'r', DisplayName='Left Wheel');
plot(df.yokePos, df.inducedPowerLossR, 'g', DisplayName='Right Wheel');
xline(referenceYoke, '--', HandleVisibility='off');
xlabel('Yoke Position [°]');
ylabel('Power @ 10m/s [W]');
legend('Location', 'best');
title('Geometry-Induced Slip Power Losses');
grid on;

% Plot 3: IC Distance (Ackermann Regions)
figure('Name', 'IC Distance', 'NumberTitle', 'off');
hold on;
plot(df.yokePos, df.ICdist, 'b', LineWidth=2, DisplayName='Current Geometry');
yline(0, 'm--', LineWidth=2, DisplayName='True Ackermann');
yline(-trackwidth, '--', LineWidth=2, DisplayName='Parallel Steer');
xline(referenceYoke, '--', HandleVisibility='off');
ylim([min([df.ICdist; -trackwidth]) - 100, max([df.ICdist; 0]) + 100]);

% Shaded regions
regions.xlim = xlim;
regions.ylim = ylim;
regions.x = [regions.xlim(1), regions.xlim(2), regions.xlim(2), regions.xlim(1)];
regions.yAA = [-trackwidth, -trackwidth, regions.ylim(1), regions.ylim(1)];
regions.yLA = [0, 0, -trackwidth, -trackwidth];
regions.yPA = [regions.ylim(2), regions.ylim(2), 0, 0];

patch(regions.x, regions.yAA, 'r', FaceAlpha=0.2, EdgeColor='none', DisplayName='"Anti-Ackermann"');
patch(regions.x, regions.yLA, 'c', FaceAlpha=0.2, EdgeColor='none', DisplayName='"Lazy Ackermann"');
patch(regions.x, regions.yPA, 'y', FaceAlpha=0.2, EdgeColor='none', DisplayName='"Pro-Ackermann"');

xlim tight;
xlabel('Yoke Position [°]');
ylabel('IC Distance [mm]');
text('String', '\it{outside whl too straight}', 'Units', 'normalized', 'Position', [1.07, 0.9]);
legend('Location', 'eastoutside');
text('String', '\it{outside whl too toed}', 'Units', 'normalized', 'Position', [1.07, 0.1]);
title('Inner Tire IC ➡ Outer Tire IC');

% Plot 4: Parasitic Camber
figure('Name', 'Parasitic Camber', 'NumberTitle', 'off');
hold on;
plot(df.yokePos, df.parasiticCamberL, 'r', DisplayName='Left Wheel');
plot(df.yokePos, df.parasiticCamberR, 'g', DisplayName='Right Wheel');
xline(referenceYoke, '--', HandleVisibility='off');
xlabel('Yoke Position [°]');
ylabel('Tire Camber [°]');
legend('Location', 'best');
title('Steering-Induced Camber');
grid on;

% Plot 5: Induced Wheel Slip Angle
figure('Name', 'Induced Slip Angle', 'NumberTitle', 'off');
hold on;
plot(df.yokePos, df.inducedAlphaL, 'r', DisplayName='Left Wheel');
plot(df.yokePos, df.inducedAlphaR, 'g', DisplayName='Right Wheel');
xline(referenceYoke, '--', HandleVisibility='off');
xlabel('Yoke Position [°]');
ylabel('Tire Slip [°]');
legend('Location', 'best');
title('Steering-Induced Wheel \alpha');
grid on;

% Plot 6: Dynamic Toe
figure('Name', 'Dynamic Toe', 'NumberTitle', 'off');
hold on;
plot(df.yokePos, df.dynamicToe, 'b', DisplayName='Current Configuration');
plot(df.yokePos, df.dynamicToeIdeal, 'm', DisplayName='True Ackermann');
xline(referenceYoke, '--', HandleVisibility='off');
xlabel('Yoke Position [°]');
ylabel('Dynamic Toe [°]');
text('String', '\it{Toe in (+)}', Position=[0.5, 0.9], Units='normalized', HorizontalAlignment='center');
text('String', '\it{Toe out (-)}', Position=[0.5, 0.1], Units='normalized', HorizontalAlignment='center');
legend('Location', 'best');
title('Dynamic Toe');
grid on;

% Plot 7: Pure Rolling Closeness
figure('Name', 'Pure Rolling Closeness', 'NumberTitle', 'off');
hold on;
plot(df.deltaOuter, df.innerError, 'b', DisplayName='Current Configuration');
yline(0, 'm--', DisplayName='True Ackermann');
xlabel('Outer Steer Angle [°]');
ylabel('% Error in Inner Steer Angle');
legend('Location', 'best');
title('Pure Rolling Closeness');
grid on;

% Plot 8: Wheel Steer Angles
figure('Name', 'Wheel Steer Angles', 'NumberTitle', 'off');
hold on;
plot(df.yokePos, df.deltaL, 'r', DisplayName='\delta Left Wheel');
plot(df.yokePos, df.deltaR, 'g', DisplayName='\delta Right Wheel');
plot(df.yokePos, df.deltaEff, 'y', DisplayName='\delta_{eff}');
plot(df.yokePos, df.betaMotionL, 'm', DisplayName='\beta Left Wheel');
plot(df.yokePos, df.betaMotionR, 'b', DisplayName='\beta Right Wheel');
xline(referenceYoke, '--', HandleVisibility='off');
xlabel('Yoke Position [°]');
ylabel('Steer Angle [°]');
legend('Location', 'best');
title('Wheel Steer');
grid on;

% Plot 9: Lateral Acceleration
figure('Name', 'Lateral Acceleration', 'NumberTitle', 'off');
plot(df.radiusEff/1e3, df.speedForLateral1G, 'k');
xlabel('Cornering Radius [m]');
ylabel('1G Corner Maxspeed [m/s]');
xlim([0 100]);
title('a_c=v^2/r Result');
grid on;

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"hidecode"}
%---
%[control:editfield:4e3c]
%   data: {"defaultValue":"0","label":"Steering arm offset [X-inset, Y-inset]","run":"SectionToEnd","valueType":"MATLAB code"}
%---
%[control:editfield:209a]
%   data: {"defaultValue":0,"label":"Extension rod connection length","run":"SectionToEnd","valueType":"Double"}
%---
%[control:editfield:9564]
%   data: {"defaultValue":0,"label":"X distance, ER axis to WC","run":"SectionToEnd","valueType":"Double"}
%---
%[control:statebutton:816f]
%   data: {"defaultValue":false,"label":"Toggle Symmetric Travel","run":"Section"}
%---
%[control:slider:24b5]
%   data: {"defaultValue":7,"label":"Resolution (2^n)","max":12,"min":4,"run":"AllSections","runOn":"ValueChanged","step":1}
%---
