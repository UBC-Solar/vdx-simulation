%% MAIN SCRIPT TO CALCULATE HARDPOINT FORCES %%
clear, close, clc, % clears workspace, closes figures, clears terminal,

% Taken from SW VDX Skeleton and CG Estimate from Alex (Sept. 2025)
totalMass = 354.37; %kg
wheelBase = 2750; %mm
COM = [128.91, 439.31, -84.24];

%%%%%% Set Loading Condition here! %%%%%%
% regs: 2g bump, 1g brake, 1g corner
% Our new standard: 4g bump, 2g brake, 1g corner
loading.bump = 2; %bump should >= 1 (no bump would be bumpG = 1 for static weight)
loading.brake = 1;
loading.corner = 1;
loading.turnDirection = 1; %1:right, -1:left

%%%%%% Set corner to compute and plot %%%%%
% Note: right side is the coordiante driving side
corner = "FR";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Display Turn Direction
if loading.turnDirection == 1
    disp("***Turning Right***")
elseif loading.turnDirection == -1
        disp("***Turning Left***")
else
    error("Invalid Turn Direction. Left: -1, Right: 1")
end

% Read front right coordinates from VDX Hardpoints spreadsheet and store in a struct, P
P.FR = getCoordinates('FRONT - SW Hardpoints');
% Negate x component to optain left side hardpoint coordinates
P.FL = structfun(@(p) [-p(1), p(2), p(3)], P.FR, "UniformOutput", false);
% Read rear right coordinates from VDX Hardpoints spreadsheet and store in a struct, P
P.RR = getCoordinates('REAR - SW Hardpoints');
% Negate x component to optain left side hardpoint coordinates
P.RL = structfun(@(p) [-p(1), p(2), p(3)], P.RR, "UniformOutput", false);

% Compute forces at tire patch and store in a struct, f
f = computeTirePatchForces(COM, totalMass, wheelBase, 2*abs(P.FR.TP(1)), loading);
disp("------Tire Patch Forces [N]------")
displayForceVectors(f)

fprintf("------Hardpoint forces for %s corner [N]------", corner)
F.(corner) = solveStaticSystemFUNC(P.(corner), f.(corner));
displayForceVectors(F.(corner))

%%%%%% Set Plot View %%%%%%
swView('iso');
% % type "help swView" in the command window for more information
% % (or look at the swView.m file in suspension) 
setupSusFig(corner)
plotLinks(P.(corner))
hold off;

% Copy Forces to be pasteable in google docs
copyForceTableForGoogleDocs(F.(corner))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% FUNCTIONS
function setupSusFig(corner)
% Sets up figure a for plotting a corner of the suspension
    cornerKeys = ["FR", "FL", "RR", "RL"];
    cornerNames = ["Front Right", "Front Left", "Rear Right", "Rear Left"];
    cornerDict = dictionary(cornerKeys, cornerNames);
    %Setting up figure
    figure(1)
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Solar VDX Suspension');
    subtitle(sprintf(['Points for %s corner', newline], cornerDict(corner)))
    axis equal
    hold on;
    legend show;
    grid on;
end
function displayForceVectors(struct)
% displayForceVectors(struct) displays force vectors in a table shown in
% the terminal.
    fields = fieldnames(struct);
    n = numel(fields);
    forceVectors = zeros(n, 3);
    for i = 1:n
        field = fields{i};
        value = struct.(field);
        forceVectors(i, :) = value;
    end
    % Create table
    disp(newline)
    disp(array2table(forceVectors, 'VariableNames', {'X', 'Y', 'Z'}, 'RowNames', fields));
    disp(newline)
end

function copyForceTableForGoogleDocs(struct)
% Forces to table functions
% Copy tab-delimited force table with headers to clipboard for Google Docs


    % Header line using sprintf for tabs
    header = sprintf('Force Name\tX (N)\tY (N)\tZ (N)\n');
    fields = fieldnames(struct);
    n = numel(fields);
    rows = strings(n,1);
    for i = 1:n
        F = struct.(fields{i});
        xStr = formatNumWithCommas(F(1));
        yStr = formatNumWithCommas(F(2));
        zStr = formatNumWithCommas(F(3));
        rows(i) = sprintf('%s\t%s\t%s\t%s', fields{i}, xStr, yStr, zStr);
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






