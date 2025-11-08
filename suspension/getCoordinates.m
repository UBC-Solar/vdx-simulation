function p = getCoordinates(sheetName)
% This script gets the coordinates from the "SW Hardpoints" Tab in the "V4 Vehicle Dynamics" Google Sheet File
% Then, it assigns coordinates to the desired variables

    % Gets hardpoints from google sheets
    ID = '1C9o_FGi18w9rInHHT_hzzjyN-1102uIo5yY6cqIHfXk';
    %sheetName = 'Copy of ALTERED FORMAT - SW Hardpoints';
    urlName = sprintf('https://docs.google.com/spreadsheets/d/%s/gviz/tq?tqx=out:csv&sheet=%s',ID, sheetName);
    data = webread(urlName);
    
    % Name Variables
    p = struct();

    % Get Symbol Column Names
    symbols = string(data.Variable);
    for i = 1:numel(symbols)
        varName = symbols(i);
        varName = replace(varName, "'", ""); % Remove single quotes if present
        varName = matlab.lang.makeValidName(varName);  % Ensure it's a valid variable name
        
        % Assigns as a field of p
        p.(varName) = [data.X(i), data.Y(i), data.Z(i)];
    end
    clear symbol varName data i ID sheetName urlName; % clear local vars

end