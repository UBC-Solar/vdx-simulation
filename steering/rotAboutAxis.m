function rotatedPt = rotAboutAxis(axisFun, point, angleRad)
%ROTABOUTAXIS Rotate a point about an axis defined by a parametric function
%   rotatedPt = rotAboutAxis(axisFun, point, angleRad) rotates a 3D point
%   about an axis defined by a parametric function axisFun.
%
%   Inputs:
%       axisFun  - Function handle @(t) that returns points on the axis [x;y;z]
%       point    - 3D point to rotate [x;y;z] (3x1 vector)
%       angleRad - Rotation angle in radians (positive = right-hand rule)
%
%   Outputs:
%       rotatedPt - Rotated 3D point [x;y;z] (3x1 vector)

arguments
    axisFun (1,1) function_handle
    point (3,1) double
    angleRad (1,1) double
end

% Get axis direction and point on axis
axisPt = axisFun(0);
axisDir = axisFun(1) - axisPt;
assert(isequal(size(axisPt), [3,1]) && norm(axisDir) > eps, 'axisFun does not define a 3D axis');

% Rodrigues' rotation formula
v = point - axisPt;         % interim shift
k = axisDir / norm(axisDir);
vRot = v * cos(angleRad) + ...
    cross(k, v) * sin(angleRad) + ...
    k * dot(k, v) * (1 - cos(angleRad));

rotatedPt = vRot + axisPt;  % shift back

end
