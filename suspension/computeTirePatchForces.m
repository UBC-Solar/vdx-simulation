function f_TP = computeTirePatchForces(COM, totalMass, wheelBase, trackWidth, loading)
    % Computes the Tire Patch Forces via static and dynamic weights.
    %%% Static weight distribution 
    % via bilinear interpolation
    g = Phys.g; % gravatiational acceleration
    
    % Normalized COM x and z coordinates
    Cx = (COM(1)+trackWidth/2)/trackWidth;
    Cz = (COM(3)+wheelBase/2)/wheelBase;
    
    % Static weights on each wheel [kg]
    wS_FL = totalMass * Cx*Cz;
    wS_FR = totalMass * (1-Cx)*Cz;
    wS_RL = totalMass * Cx*(1-Cz);
    wS_RR = totalMass * (1-Cx)*(1-Cz);
    
    %%% Dynamic Weights
    % Due to pitching moment
    brakeLoadTransferFraction = COM(2)/wheelBase;
    % Due to rolling mooment
    cornerLoadTransferFraction = COM(2)/trackWidth;
    
    % Load transfer magnitudes
    brakeLoadTransfer = loading.brake * brakeLoadTransferFraction;
    cornerLoadTransfer = loading.corner * cornerLoadTransferFraction;
    
    % Dynamic Weights [kg]
    wD_FL = wS_FL * (1 + brakeLoadTransfer) * (1 + cornerLoadTransfer * loading.turnDirection); 
    wD_FR = wS_FR * (1 + brakeLoadTransfer) * (1 - cornerLoadTransfer * loading.turnDirection); 
    wD_RL = wS_RL * (1 - brakeLoadTransfer) * (1 + cornerLoadTransfer * loading.turnDirection); 
    wD_RR = wS_RR * (1 - brakeLoadTransfer) * (1 - cornerLoadTransfer * loading.turnDirection);
    
    % Tire Patch Forces [N]
    f_TP.FL = wD_FL * g .* [loading.corner, loading.bump, loading.brake];
    f_TP.FR = wD_FR * g .* [loading.corner, loading.bump, loading.brake];
    f_TP.RL = wD_RL * g .* [loading.corner, loading.bump, loading.brake];
    f_TP.RR = wD_RR * g .* [loading.corner, loading.bump, loading.brake];
end