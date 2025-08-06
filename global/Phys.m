classdef Phys
    % Phys Common physical constants used in vehicle modeling.
    %   Values are given as doubles with metric units. 
    %
    %   Phys is a static class and should not be instantiated.
    %   Access constants directly using the class name, e.g.:
    %       force = mass * Phys.g;

    properties (Constant)
        g = 9.80665         % m/s²
        rho = 1.225         % kg/m³
    end

    methods (Access = private)
        function obj = Phys()
            % prevent instantiation
        end
    end
end
