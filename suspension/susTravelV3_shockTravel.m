%% MAIN SCRIPT TO CALCULATE HARDPOINT FORCES %%
clear, close, clc, % clears workspace, closes figures, clears terminal,

% Taken from SW VDX Skeleton and CG Estimate (April 23,2025)
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
corner = "FL";

%%%%%% Set Plot View %%%%%%
%sw_view('right');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Display Turn Direction
if loading.turnDirection == 1
    disp("***Turning Right***")
elseif loading.turnDirection == -1
        disp("***Turning Left***")
else
    error("Invalid Turn Direction. Left: -1, Right: 1")
end

% Read coordinates from VDX Hardpoints spreadsheet and store in a struct, P
P.FR = getCoordinates('FRONT - SW Hardpoints');
% Negate x component to optain left side hardpoint coordinates
P.FL = structfun(@(p) [-p(1), p(2), p(3)], P.FR, "UniformOutput", false);

% Read coordinates from VDX Hardpoints spreadsheet and store in a struct, P
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

% 
% % BUMPING %%%%%%%%%%%%%%%%%%%%
% % 2 inch (50.8mm) bump case
% resolution = 25;%%
% numOfOutputs = 6;
% MAX_BUMP_DISPLACMENT = 2; %[in]
% wheelDisplacement =  MAX_BUMP_DISPLACMENT*25.4; %[mm]
% wheelDisps = linspace(0,wheelDisplacement, resolution);
% dh = wheelDisplacement/resolution;
% F_mags = zeros(numOfOutputs,resolution);
% for idx = 1:resolution
%     %pause(0.01)
% 
%     p_TP = p_TP + [0,dh,0];
%     p_WC = p_WC + [0,dh,0];
% 
%     pU_UCA = rotatePoint(pU_UCA, pC_UCA_in, dh);
%     pU_LCA = rotatePoint(pU_LCA, pC_LCA_in, dh);
%     pUCA_PR = rotatePoint(pUCA_PR, pR_C, dh);
%     pR_PR = rotatePoint(pR_PR, pR_C, dh);
%     pR_S = rotatePoint(pR_S, pR_C, dh);
%     pTR_out = rotatePoint(pTR_out, pTR_in, dh);
%     %implement projection to find piot point of other points
%     %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 
% %% Computes Front Suspension Forces and Plots
% 
% %%% COMPUTE FORCES HERE AND RECORD
% 
%     % Recording Forces
%     F_mags(:,idx) = F_mag;


%%%%%% Set Plot View %%%%%%
sw_view('iso');
% type "help sw_view" in the command window for more information
% (or look at the sw_view.m file in suspension) 
setupSusFig(corner)
plotLinks(P.(corner))
hold off;

copyForceTableForGoogleDocs(F.(corner))



% for j = 1:numOfOutputs
%     plot(wheelDisps/25.4,F_mags(j,:)/1000, 'DisplayName', forceNames{j})
%     if j == 1
%         hold on
%         legend show;
%         xlabel("Wheel Displacement [in]")
%         ylabel("Signed Force Magnitude [kN]")
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%

%% FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%
function setupSusFig(corner)
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

% function displayVectorComponents(vectorNames)
%     n = numel(vectorNames);
%     components = zeros(n, 3);  % Preallocate matrix
% 
%     for i = 1:n
%         forceVector(i, :) = evalin('base', vectorNames{i});
%     end
% 
%     % Create table
%     disp(array2table(components, 'VariableNames', {'X', 'Y', 'Z'}, 'RowNames', vectorNames));
% end

%% Forces to table functions
function copyForceTableForGoogleDocs(struct)
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






