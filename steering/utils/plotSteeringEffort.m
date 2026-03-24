function plotSteeringEffort(torqueMatrix, lateralAccelMatrix, kph, plotColors, centerIdx, figTitle)
% plotSteeringEffort  Open a figure and plot a steering effort curve family.
%
%   torqueMatrix      - [numPoints x numVelCurves] torque data [N·m]
%   lateralAccelMatrix- [numPoints x numVelCurves] lateral accel data [m/s²]
%   kph               - [1 x numVelCurves] speed labels [km/h]
%   plotColors        - [numVelCurves x 3] RGB color array
%   centerIdx         - Row index of the straight-ahead (zero rack) point
%   figTitle          - String title for this figure

figure(Name=figTitle); hold on;
title(figTitle);
xlabel('Lateral Acceleration [m/s²]');
ylabel('Steering Wheel Torque [N·m]');
grid on;
xlim([-10, 10]);
ylim([-5, 5]);
set(gca, XDir='reverse');
yline(0, '--', Color=[0.5 0.5 0.5]);

numVelCurves = length(kph);
plotLines = gobjects(numVelCurves, 1);

for j = numVelCurves:-1:1
    plotLines(j) = plot(lateralAccelMatrix(:, j), torqueMatrix(:, j), ...
        Color=plotColors(j, :), ...
        LineWidth=2, ...
        DisplayName=sprintf('%4.1f km/h', kph(j)));
end

% Static torque marker at straight-ahead
if ~isempty(centerIdx)
    Tstatic = torqueMatrix(centerIdx, 1);
    plotDot = plot(0, Tstatic, 'ko', MarkerFaceColor='k', MarkerSize=6, ...
        DisplayName=sprintf('%4.2f N·m', Tstatic));
    legend([plotLines; plotDot], Location='northeast');
else
    legend(plotLines, Location='northeast');
end
