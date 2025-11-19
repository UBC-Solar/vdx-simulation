function f_TP = computeTirePatchForces(COM, totalMass, wheelBase, trackWidth, loading)
%Compute tire patch force vectors under loading.
%
%   f_TP = computeTirePatchForces(COM, totalMass, wheelBase, trackWidth, loading)
%   computes the tire patch force vectors for each wheel by combining:
%       • static weight distribution (based on the vehicle center of mass)
%       • dynamic load transfer due to braking and cornering
%
%   Forces are returned in Newtons and resolved in the order:
%       [ lateral (corner), vertical (bump), longitudinal (brake) ].
%
%   INPUTS:
%       COM         (1×3 vector)
%           Center of mass location relative to vehicle origin [x, y, z].
%           x: lateral direction (positive = left)
%           y: vertical / height above ground
%           z: longitudinal direction (positive = front)
%
%       totalMass   (scalar)
%           Total vehicle mass [kg].
%
%       wheelBase   (scalar)
%           Distance between front and rear axles [m].
%
%       trackWidth  (scalar)
%           Lateral distance between left and right wheels [m].
%
%       loading     (struct)
%           Contains dynamic loading inputs:
%               loading.brake          — Longitudinal braking load factor
%               loading.corner         — Lateral cornering load factor
%               loading.bump           — Vertical bump load factor
%               loading.turnDirection  — +1 for left turn, -1 for right turn
%
%   OUTPUT:
%       f_TP        (struct)
%           Tire patch force vectors for each wheel (fields FL, FR, RL, RR).
%           Each field is a 1×3 vector of forces [N]:
%               [F_lat, F_vert, F_long]
%
%   METHOD:
%       1. Static weight distribution is computed via bilinear interpolation
%          using COM.x and COM.z relative to wheelbase and trackwidth.
%
%       2. Dynamic load transfer is computed using:
%           - Pitch load transfer ~ COM.y / wheelBase
%           - Roll load transfer  ~ COM.y / trackWidth
%
%       3. Static and dynamic weights are multiplied by gravity and
%          scaled by the loading factors to produce tire force vectors.
%
%   NOTES:
%       - Requires Phys.g to be defined in the path (gravitational constant).
%       - All returned forces include the dynamic scaling from loading.*.
%
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