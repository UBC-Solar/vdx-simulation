%% Computes forces at all four tire patches
% Run mainHardpointForces.m! this script should not be run on own
x = 1; y = 2; z = 3;
%% Loading Condition
% %regs: 1g turn, 2g bump. 1g braking
% bumpG = 2 %bump should >= 1 (no bump would be bumpG = 1 for static weight)
% brakeG = 1
% cornerG = 1
% turnDirection = 1;
% % 1 right, -1 left
% %disp(['(Turing Right)', newline])


%% Static weight distribution 
% via bilinear interpolation

% Normalized COM x and z coordinates
Cx = (COM(x)+trackWidth/2)/trackWidth;
Cz = (COM(z)+wheelBase/2)/wheelBase;

% Static weights on each wheel [kg]
wS_FL = totalMass * Cx*Cz;
wS_FR = totalMass * (1-Cx)*Cz;
wS_RL = totalMass * Cx*(1-Cz);
wS_RR = totalMass * (1-Cx)*(1-Cz);

% Conservation of mass check
% isequal((wS_FL + wS_FR + wS_RL + wS_RR), totalMass)

%% Dynamic Weights

% Due to pitching moment
brakeLoadTransferFraction = COM(y)/wheelBase;
% Due to rolling mooment
cornerLoadTransferFraction = COM(y)/trackWidth;

% Load transfer magnitudes
brakeLoadTransfer = brakeG * brakeLoadTransferFraction;
cornerLoadTransfer = cornerG * cornerLoadTransferFraction;

% Dynamic Weights [kg]
wD_FL = wS_FL * (1 + brakeLoadTransfer) * (1 + cornerLoadTransfer * turnDirection); 
wD_FR = wS_FR * (1 + brakeLoadTransfer) * (1 - cornerLoadTransfer * turnDirection); 
wD_RL = wS_RL * (1 - brakeLoadTransfer) * (1 + cornerLoadTransfer * turnDirection); 
wD_RR = wS_RR * (1 - brakeLoadTransfer) * (1 - cornerLoadTransfer * turnDirection);

% Tire Patch Forces [N]
f_FL_TP = wD_FL * g .* [cornerG, bumpG, brakeG];
f_FR_TP = wD_FR * g .* [cornerG, bumpG, brakeG];
f_RL_TP = wD_RL * g .* [cornerG, bumpG, brakeG];
f_RR_TP = wD_RR * g .* [cornerG, bumpG, brakeG];
