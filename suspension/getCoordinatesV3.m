% This sc'ript geterives the coordinates from the "Carsim Hardpoints" Tab in the "V4 Vehicle Dynamics" Google Sheet File
% Then, it assigns coordinates to the desired variables
clear,close,clc

% Gets hardpoints from google sheets
ID = '1C9o_FGi18w9rInHHT_hzzjyN-1102uIo5yY6cqIHfXk';
sheetName = 'SW Hardpoints';
urlName = sprintf('https://docs.google.com/spreadsheets/d/%s/gviz/tq?tqx=out:csv&sheet=%s',ID, sheetName);
data = webread(urlName);

% Name Variables
symbols = string(data.Variable); % Get symbol column names
for i = 1:numel(symbols)
    varName = symbols(i);
    varName = replace(varName, "'", ""); % Remove single quotes if present
    varName = matlab.lang.makeValidName(varName);  % Ensure it's a valid variable name
    assignin('base',varName,[data.X(i), data.Y(i), data.Z(i)])
end
clear symbol varName data i ID sheetName urlName symbols; % clear local vars