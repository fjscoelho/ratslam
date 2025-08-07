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
GT_table = readtable('exported_data/gps.csv');
n2 = height(GT_table);
lat = table2array(GT_table(1:n2,"latitude"));
long = table2array(GT_table(1:n2,"longitude"));

% interpolate zero datas
for i = 1:n2
    if long(i) == 0
        long(i) = (long(i-1)+long(i+1))/2;
    end
    if lat(i) == 0
        lat(i) = (lat(i-1)+lat(i+1))/2;
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



