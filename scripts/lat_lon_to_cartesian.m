function [x, y] = lat_lon_to_cartesian(latitudes, longitudes)
    
% Convert geographical coordinates (lat/lon) to plane Cartesian coordinates (x,y)
% in meters, relate to first point of the series.
%
% Sintax:
%   [x, y] = lat_lon_to_cartesian(latitudes, longitudes)
%
% Input:
%   latitudes  - latitude vector ind decimal deg
%   longitudes - longitude vector ind decimal deg
%
% Output:
%   x - x coordinates in meters (East-Wester axis)
%   y - y coordinates in meters (North-South axis)

    % Mean radius of Earth (WGS84)
    R = 6378137;
    
    %To radians
    lat_rad = deg2rad(latitudes);
    lon_rad = deg2rad(longitudes);
    
    % Reference Point (first point)
    lat0 = lat_rad(1);
    lon0 = lon_rad(1);
    
    % Difference to first point
    delta_lat = lat_rad - lat0;
    delta_lon = lon_rad - lon0;
    
    % To Cartesian Coordinates
    y = R * delta_lat;  % North-South
    x = R * delta_lon .* cos(lat0);  % (East-Wester axis) (with latitude correction)
end