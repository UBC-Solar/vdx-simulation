function q = getPtOnAxis(p1, p2, h)
%getPtOnAxis  Intersection of a line through two points with horizontal plane
%   q = getPtOnAxis(p1,p2,h) returns the 3x1 point on the line through p1 and
%   p2 with z == h.

z1 = p1(3);
z2 = p2(3);

if z1 == z2
    error('getPtOnAxis() requires points with unique z')
end

t = (h - z1) / (z2 - z1);

q = p1 + t*(p2 - p1);
q(3) = h;
end

