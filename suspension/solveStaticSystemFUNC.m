function hardpointForces = solveStaticSystemFUNC(hardpointCoordinates, tirePatchForces)
% Computes the hardpoint forces for a quadrant (corner) of the suspension
    p = hardpointCoordinates;
    f_TP = tirePatchForces;
    %% Direction Vectors
    u_tieRod = p.TR_in - p.TR_out;
    u_LCA_in = p.C_LCA_in - p.U_LCA; %inboard was front and outboard was rear
    u_LCA_out = p.C_LCA_out - p.U_LCA; 
    u_UCA_in = p.C_UCA_in - p.U_UCA;
    u_UCA_out = p.C_UCA_out - p.U_UCA;
    u_PR = p.R_PR - p.UCA_PR;
    
    % Normalizing;
    u_tieRod = u_tieRod/norm(u_tieRod);
    u_LCA_in = u_LCA_in/norm(u_LCA_in);
    u_LCA_out = u_LCA_out/norm(u_LCA_out);
    u_UCA_in = u_UCA_in/norm(u_UCA_in);
    u_UCA_out = u_UCA_out/norm(u_UCA_out);
    u_PR = u_PR/norm(u_PR);
    
    %% Calculate Moment Arms from each chassis point
    MC = p.O; % moment center setting as origin for now
    %WILL NEED TO BE A DATA FIELD LATER FOR CORNER (SINCE DIFFERNET FOR FRONT AND BACK)
    d_tieRod = (p.TR_in - MC);
    d_LCA_in = (p.C_LCA_in - MC);
    d_LCA_out = (p.C_LCA_out - MC);
    d_UCA_in = (p.C_UCA_in - MC);
    d_UCA_out = (p.C_UCA_out - MC);
    d_PR = (p.R_PR - MC);
    
    
    %% Calculate momemnt unit vectors for each chassis point 
    % using the moment arms and force unit vectors
    uM_tieRod = cross(d_tieRod, u_tieRod);
    uM_LCA_in = cross(d_LCA_in, u_LCA_in);
    uM_LCA_out = cross(d_LCA_out, u_LCA_out);
    uM_UCA_in = cross(d_UCA_in, u_UCA_in);
    uM_UCA_out = cross(d_UCA_out, u_UCA_out);
    uM_PR = cross(d_PR, u_PR);
    
    %% Calcualte moment due to input force at tire contact patch
    d_TP = (p.TP - MC);
    M_TP = cross(d_TP, f_TP);
    
    %% Solve the system (Ax = b)
    
    A = [u_tieRod', u_LCA_in', u_LCA_out', u_UCA_in', u_UCA_out', u_PR';
        uM_tieRod', uM_LCA_in', uM_LCA_out', uM_UCA_in', uM_UCA_out', uM_PR'];
    
    b = [-f_TP';
        -M_TP'];
    
    F_mag = A\b;
    
    %% Scale Unit Vectors by force magnitude to obtain force vectors
    F.tieRod = F_mag(1) * u_tieRod;
    F.LCA_in = F_mag(2) * u_LCA_in;
    F.LCA_out = F_mag(3) * u_LCA_out;
    F.UCA_in = F_mag(4) * u_UCA_in;
    F.UCA_out = F_mag(5) * u_UCA_out;
    F.PR = F_mag(6) * u_PR;

    %% Running Rocker Routine
    % Rocker Force Analysis
    % Run mainHardpointForces.m! this script should not be run on own
    % Assumming rocker plane is parallel to the YZ plane (x constant)

    if F.PR(1) ~= 0
        error(['ValueError: F.PR(1) must be zero since F.PR should be ' ...
            'parallel to the rocker plane, but F.PR(1) = %g.'], F.PR(1));
    end
    
    % Shock unit direction vector
    u_S = (p.C_S - p.R_C)/norm(p.C_S - p.R_C);
    
     % Moment about rocker chassis point
     dR_PR = p.R_PR - p.R_C;
     dR_S = p.R_S - p.R_C;
    
     % Unit moment vectors with respect to the chasis pivot
     uMR_S = cross(dR_S, u_S);

     % Moment due to pull rode w.r.t chasis pivot
     MR_PR = cross(dR_PR, F.PR);
    
     % Linear System
     AR = [1 0 u_S(2);
           0 1 u_S(3);
           0 0 uMR_S(1)];

     % THINK ABOUT THISE AR MORE INTERMS OF THE DIMENSION AND MOMENT ARM
     % DIRECTIONS
    
     bR = [-F.PR(2);
           -F.PR(3);
           -MR_PR(1)];

    FR = AR\bR;
    
    % Rocker-Chassis Force, Rocker Shock Force
    F.RC = [0, FR(2), FR(3)];
    F.S = FR(3)*u_S;

   % Output Hardpoint Forces
   hardpointForces = F;
end









