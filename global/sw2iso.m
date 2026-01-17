function isoCoords = sw2iso(swCoords)
%sw2iso Convert SolidWorks coordinates to ISO 8855
%   TODO describe orientation

isoCoords = [swCoords(3); swCoords(1); swCoords(2)];
end
