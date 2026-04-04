function verifyCarConstants(totalMass, wheelBase, COM)
% verifyCarConstants(totalMass, wheelBase, COM)
%   Verify script constants against CarV4 properties
%   to avoid diverging.
%
%   Issues warning if mismatch or CarV4 not found.
%
%   Interim solution.

if ~isempty(which('CarV4'))
    car = CarV4();
    cogExpected = [(car.CoGy - 0.5)*car.Trackwidth, car.CoGh, (car.CoGx - 0.5)*car.Wheelbase];
    % in SolidWorks coords
    if ~(totalMass == car.Mass && wheelBase == car.Wheelbase && norm(COM - cogExpected) < 1e-3*norm(cogExpected))
        warning('Script constants do not match CarV4.');
    end
else
    warning('CarV4 not found on path - constants unverified.');
end
end
