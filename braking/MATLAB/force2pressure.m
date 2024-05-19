function pressure = force2pressure(force, diameter, varargin)
% FORCE2PRESSURE - to convert an input force into a pressure value for a
% given diameter piston bore. Input units must be consistent

checkMatrix = ["imperial", "imperia", "imperi", "imper", "impe", "imp", "im", "i"];

inputUnits = "NULL";
outputUnits = "NULL";

if (numel(varargin))
    inputUnits = varargin(1);
    inputUnits = lower(inputUnits{:});
    outputUnits = varargin(2);
    outputUnits = lower(outputUnits{:});
end

PAtoPSI = 6895;
pressureModifier = 1;

if (ismember(outputUnits, checkMatrix)) pressureModifier = PAtoPSI; end
if (ismember(inputUnits, checkMatrix)) pressureModifier = 1; end

area = (1/4)*pi*(diameter)^2;
pressure = (force/(area))/pressureModifier;

end
