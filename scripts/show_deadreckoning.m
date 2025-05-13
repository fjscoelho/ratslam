%% Show the robot's odometry

clear
close all;
clc;

% Dead Reckoning
pose_table = readtable('exported_data/pose.csv');
n = height(pose_table);
pose_x = table2array(pose_table(1:n,"pos_x"));
pose_y = table2array(pose_table(1:n,"pos_y"));


plot(pose_x,pose_y,'LineWidth',2)
hold on
scatter(pose_x(1),pose_y(1),'green','filled','LineWidth',2)
scatter(pose_x(end),pose_y(end),'red','square','filled','LineWidth',2)
hold off
title('Dead Reckoning','FontSize',12,'Interpreter','latex')
xlabel('$x$ (m)','FontSize',12,'Interpreter','latex');
ylabel('$y$ (m)','FontSize',12,'Interpreter','latex');
legend('trajectory','initial position','final position','Interpreter','latex','Location','best')
grid on

print('-dpng', '-r600', 'Figures/dead_reckoning.png');
print('-depsc2', '-r600', 'Figures/dead_reckoning.eps');

% Ground Truth
GT_table = readtable('exported_data/ground_truth.csv');
n2 = height(GT_table);
lat = table2array(GT_table(1:n2,"latitude"));
long = table2array(GT_table(1:n2,"longitude"));

% interpolate zero datas
for i = 1:n2
    if long(i) == 0
        long(i) = (long(i-1)+long(i+1))/2
    end
    if lat(i) == 0
        lat(i) = (lat(i-1)+lat(i+1))/2
    end
end

[x, y] = lat_lon_to_cartesian(lat, long);

figure'
plot(x,y,'LineWidth',2)
hold on
scatter(x(1),y(1),'green','filled','LineWidth',2)
scatter(x(end),y(end),'red','square','filled','LineWidth',2)
hold off
title('Ground Truth','FontSize',12,'Interpreter','latex')
xlabel('$x$ (m)','FontSize',12,'Interpreter','latex');
ylabel('$y$ (m)','FontSize',12,'Interpreter','latex');
legend('trajectory','initial position','final position','Interpreter','latex','Location','best')
grid on

print('-dpng', '-r600', 'Figures/Ground_truth.png');
print('-depsc2', '-r600', 'Figures/Ground_truth.eps');

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

