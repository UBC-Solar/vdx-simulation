function intercept = getICintercept(tirePos, steerAngle)
% getICintercept - Calculate lateral distance to instantaneous center
%
%   intercept = getICintercept(tirePos, steerAngle)
%
%   Returns lateral distance to IC along vehicle X-axis (rear axle line).
%
%   Inputs:
%       tirePos     - Tire contact patch position [2x1] or [3x1] (X, Y, ...)
%       steerAngle  - Wheel steer angle [rad]
%
%   Output:
%       intercept   - Lateral position of IC [mm]
%
%   See also: getDelta, genPerformanceTable

arguments
    tirePos (:,1) double
    steerAngle (1,1) double
end

intercept = tirePos(2) + tirePos(1) * cot(steerAngle);
end
