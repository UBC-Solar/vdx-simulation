function pathFun = getRevolvePath(axisFun, point)
%GETREVOLVEPATH Get parametric function defining circular revolution path
%   pathFun = getRevolvePath(axisFun, point) returns a function
%   handle that defines the circular path a point traces as it revolves
%   around an axis.
%
%   Inputs:
%       axisFun  - Function handle @(t) that returns points on the axis [x;y;z]
%       point    - 3D point to revolve [x;y;z] (3x1 vector)
%
%   Outputs:
%       pathFun  - Function handle @(t) where t is angle in radians
%                  Returns position on revolution path [x;y;z]

arguments
    axisFun (1,1) function_handle
    point (3,1) double
end

% Get axis direction and point on axis
axisPt = axisFun(0);
axisDir = axisFun(1) - axisPt;
assert(isequal(size(axisPt), [3,1]) && norm(axisDir) > eps, 'axisFun does not define a 3D axis');

% Rodrigues' rotation formula
v = point - axisPt;         % interim shift
k = axisDir / norm(axisDir);

% Return parametric function defining circle
pathFun = @(t) v * cos(t) + cross(k, v) * sin(t) + k * dot(k, v) * (1 - cos(t)) + axisPt;

end
