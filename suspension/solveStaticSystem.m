%% Computes Front Suspension Forces and Plots

% Determining if we are computing the front left or right 
if p_WC(1) < 0
    f_TP = f_FR_TP;
else
    f_TP = f_FL_TP;
end


% 2 inch (50.8mm) bump case
% MAX_BUMP_DISPLACMENT = 2; %[in]
% wheelDisplacement = MAX_BUMP_DISPLACMENT*25.4; %[mm]
% 
% pU_UCA = rotatePoint(pU_UCA, pC_UCA_in, wheelDisplacement);
% pU_LCA = rotatePoint(pU_LCA, pC_LCA_in, wheelDisplacement);
% pUCA_PR = rotatePoint(pUCA_PR, pR_C, wheelDisplacement);
% pR_PR = rotatePoint(pR_PR, pR_C, wheelDisplacement);
% pR_S = rotatePoint(pR_S, pR_C, wheelDisplacement);
%pTR_out = rotatePoint


%ZZZ TESTING
% XXX = norm(pU_UCA - pC_UCA_in);
% p_0 = pU_UCA;
% pU_UCA = rotatePoint(pU_UCA, pC_UCA_in, 3);
% YYY = norm(p_0-pU_UCA);
% ANGLE = atand(YYY/XXX)
% ANGLE2 = atand(3/XXX)

%% Direction Vectors
u_tieRod = pTR_in - pTR_out;
u_LCA_in = pC_LCA_in - pU_LCA; %inboard was front and outboard was rear
u_LCA_out = pC_LCA_out - pU_LCA; 
u_UCA_in = pC_UCA_in - pU_UCA;
u_UCA_out = pC_UCA_out - pU_UCA;
u_PR = pR_PR - pUCA_PR;

% Normalizing;
u_tieRod = u_tieRod/norm(u_tieRod);
u_LCA_in = u_LCA_in/norm(u_LCA_in);
u_LCA_out = u_LCA_out/norm(u_LCA_out);
u_UCA_in = u_UCA_in/norm(u_UCA_in);
u_UCA_out = u_UCA_out/norm(u_UCA_out);
u_PR = u_PR/norm(u_PR);

%% Calculate Moment Arms from each chassis point
MC = p_O; % moment center setting as origin for now
%WILL NEED TO BE A DATA FIELD LATER FOR CORNER (SINCE DIFFERNET FOR FRONT AND BACK)
d_tieRod = (pTR_in - p_O);
d_LCA_in = (pC_LCA_in - p_O);
d_LCA_out = (pC_LCA_out - p_O);
d_UCA_in = (pC_UCA_in - p_O);
d_UCA_out = (pC_UCA_out - p_O);
d_PR = (pR_PR - p_O);


%% Calculate momemnt unit vectors for each chassis point 
% using the moment arms and force unit vectors
uM_tieRod = cross(d_tieRod, u_tieRod);
uM_LCA_in = cross(d_LCA_in, u_LCA_in);
uM_LCA_out = cross(d_LCA_out, u_LCA_out);
uM_UCA_in = cross(d_UCA_out, u_UCA_in);
uM_UCA_out = cross(d_UCA_out, u_UCA_out);
uM_PR = cross(d_PR, u_PR);

%% Calcualte moment due to input force at tire contact patch
d_TP = (p_TP - MC);
M_TP = cross(d_TP, f_TP);

%% Solve the system (Ax = b)

A = [u_tieRod', u_LCA_in', u_LCA_out', u_UCA_in', u_UCA_out', u_PR';
    uM_tieRod', uM_LCA_in', uM_LCA_in', uM_UCA_in', uM_UCA_out', uM_PR'];

b = [-f_TP';
    -M_TP'];

F_mag = A\b;

%% Scale Unit Vectors by force magnitude to obtain force vectors
F_tieRod = F_mag(1) * u_tieRod;
F_LCA_in = F_mag(2) * u_LCA_in;
F_LCA_out = F_mag(3) * u_LCA_out;
F_UCA_in = F_mag(4) * u_UCA_in;
F_UCA_out = F_mag(5) * u_UCA_out;
F_PR = F_mag(6) * u_PR;

%zzz_F_sum = F_tieRod+F_LCA_in+F_LCA_out+F_UCA_in+F_UCA_out+F_PR

%% Running Rocker Script
run('rockerForcesV1.m')

%% Put Forces in a table
forces = [
    F_tieRod;
    F_LCA_in;
    F_LCA_out;
    F_UCA_in;
    F_UCA_out;
    F_PR;
    F_RC;
    F_S];

forceNames = {
    'F_tieRod';
    'F_LCA_in';
    'F_LCA_out';
    'F_UCA_in';
    'F_UCA_out';
    'F_PR';
    'F_RC';
    'F_S'
};


%% Rotate a point about a an axis parallel to X axis
function P_rotated = rotatePoint(point, axisPoint, displacement)

    % Shift Point to Origin
    P_shifted = point - axisPoint;

    % Rotation matrix about X axis --FIX! cooked cooked 
    t = asin(displacement/norm(point-axisPoint));
    Rx = [ 1,       0,        0;
           0,     cos(t)   sin(t);
           0,      -sin(t)   cos(t)];

    % Rotate shifted point about origin
    P_rotated_shifted = Rx * P_shifted';

    P_rotated = P_rotated_shifted' + axisPoint; 
end









