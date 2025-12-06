function p_min = ringPointMinZ(c, v, R)
% Computes the point on a ring around axis v with minimal z

v_hat = v / norm(v);          % normalize axis
u = cross(v_hat, [0;0;1]);    % vector perpendicular to axis
u = u / norm(u);
w = cross(v_hat, u);          % second orthogonal vector

theta = atan2(-w(3), -u(3));  % angle for minimum z
p_min = c + R*(cos(theta)*u + sin(theta)*w);
end
