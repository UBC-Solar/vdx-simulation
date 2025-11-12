function bumpTransform(hardpoints, displacement, plot, pauseTime)
%IN PROGRESS
% 2 inch (50.8mm) bump case
    resolution = 25;%%
    numOfOutputs = 6;
    MAX_BUMP_DISPLACMENT = 2; %[in]
    wheelDisplacement =  MAX_BUMP_DISPLACMENT*25.4; %[mm]
    %wheelDisps = linspace(0,wheelDisplacement, resolution);
    dh = wheelDisplacement/resolution;
    F_mags = zeros(numOfOutputs,resolution);
    for idx = 1:resolution
        %pause(0.01)
    
        p_TP = p_TP + [0,dh,0];
        p_WC = p_WC + [0,dh,0];
    
        pU_UCA = rotatePoint(pU_UCA, pC_UCA_in, dh);
        pU_LCA = rotatePoint(pU_LCA, pC_LCA_in, dh);
        pUCA_PR = rotatePoint(pUCA_PR, pR_C, dh);
        pR_PR = rotatePoint(pR_PR, pR_C, dh);
        pR_S = rotatePoint(pR_S, pR_C, dh);
        pTR_out = rotatePoint(pTR_out, pTR_in, dh);
    end
end
        %implement projection to find piot point of other points
%     %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 
% %% Computes Front Suspension Forces and Plots
% 
% %%% COMPUTE FORCES HERE AND RECORD
% 
%     % Recording Forces
%     F_mags(:,idx) = F_mag;






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
