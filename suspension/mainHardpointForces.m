%% MAIN SCRIPT TO CALULATE HARDPOINT FORCES %%
clear, close, clc, clf % clears workspace, closes figures, clears terminal, clear figure
x = 1; y = 2; z = 3; % for clarity

%%%%%% Set Loading Condition here! %%%%%%
%regs: 1g turn, 2g bump, 1g braking
bumpG = 2; %bump should >= 1 (no bump would be bumpG = 1 for static weight)
brakeG = 1;
cornerG = 1;
turnDirection = -1; %right
%turnDirection = -1 %left

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
% DO NOT EDIT BELOW:
% Running Routines to Obtain Hardpoint Forces %

% Getting coordinates from google sheet SW Hardpoints tab of V4 Vehicle
% Dynamics sheet
run("getCoordinatesV3.m")

% Setting up figure
figure(1)
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Solar VDX Suspension');
subtitle(sprintf('Front %s Side(s)', side))
axis equal
hold on;
legend show;
grid on;

scatter3(COM(1), COM(2), COM(3), 'rx', 'DisplayName', 'COM')

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



