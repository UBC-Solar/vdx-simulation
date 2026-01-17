function delta = getDelta(wheelCenterPt, spindlePt)
% getDelta - Determine wheel heading angle from geometry
%
%   delta = getDelta(wheelCenterPt, spindlePt)
%
%   Calculates the steer angle of a wheel based on the spindle orientation.
%
%   Inputs:
%       wheelCenterPt   - 3D wheel center position [3x1]
%       spindlePt       - 3D spindle end position [3x1]
%
%   Output:
%       delta           - Steer angle [rad], positive = left turn
%
%   See also: solveSteeringLinkage, getICintercept

arguments
    wheelCenterPt (3,1) double
    spindlePt (3,1) double
end

spindleVec = wheelCenterPt - spindlePt;
up = [0; 0; 1];
fwd = [1; 0; 0];

heading = cross(spindleVec, up);
heading = heading / norm(heading);
if heading(1) < 0
    heading = -heading;
end

angleMag = acos(dot(heading, fwd));
if angleMag <= eps
    delta = 0;
else
    angleDir = sign(dot(cross(fwd, heading), up));  % Z component of cross
    delta = angleDir * angleMag;
end
end
