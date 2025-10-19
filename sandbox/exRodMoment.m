% estimate moment in extension rod from tie rod

Fz = -2188.9; % N
R = 50;       % mm
L = 250;      % mm
T = 2;        % in
s = 0.25;

[M, r, theta] = getM(abs(Fz), R/1e3, L/1e3, T*25.4/1e3, s)

function [M, r, theta] = getM(F, R, L, T, s)
foo = T*(1-s)/L;
theta = atand(foo);

r = R*foo/sqrt(foo^2+1);
M = F*r;
end
