%% Rocker Force Analysis
% Run mainHardpointForces.m! this script should not be run on own
% Assumming rocker plane is parallel to the YZ plane (x constant)

% Pull Rod unit vector
u_PR = F_PR/norm(F_PR);

% Shock unit direction vector
u_S = (pC_S - pR_C)/norm(pC_S - pR_C);

 % Moment about rocker chassis point
 dR_PR = pR_PR - pR_C;
 dR_S = pR_S - pR_C;

 % Unit moment vectors
 uMR_PR = cross(dR_PR, u_PR);
 uMR_S = cross(dR_S, u_S);

 % Linear System
 AR = [1, 0, u_S(y);
      0, 1, u_S(z);
      zeros(3,1), zeros(3,1), uMR_S'];

 bR = [-F_PR(y);
     -F_PR(z);
     -(norm(F_PR)*uMR_PR)'];

FR = AR\bR;

% Rocker-Chassis Force, Rocker Shock Force
F_RC = [0, FR(1), FR(2)];
F_S = FR(3)*u_S; 

