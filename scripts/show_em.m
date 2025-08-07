%% Show Experience Map

clear
close all;
clc;

% Set to true to save a experience map evolution video
save_video = false;

% Set to true to save partial figures of map evolution
save_figures = false;

if save_video
    numFrames = 50; % Número de iterações/timesteps
    videoFile = 'Figures/Exp_Map/experience_map_evolution.avi';
    vidObj = VideoWriter(videoFile);
    vidObj.FrameRate = 4;
    vidObj.Quality = 100; 
    open(vidObj);
end

% Load experience map data
nodes = readtable('exported_data/nodes.csv');
links = readtable('exported_data/links.csv');

% Load Ground Truth
GT_table = readtable('exported_data/gps.csv');

% % Get the last time stamp of experience map nodes
% last_node_time_stamp = table2array((nodes(end,"stamp_sec")))
% 
% % find time_stamp in GT_table
% [found, position] = ismember(last_node_time_stamp, GT_table.stamp_sec);
% if found
%     disp(position)
%     linha_encontrada = GT_table(position, :)
% end
% 
% % cut gps table to last map time stamp
% GT_table = GT_table(1:position, :);

% extract lat and long
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

n = height(nodes);
nodes_x = [0];
nodes_y = [0];
id = 0;

min_x = min([x; nodes.x]);
max_x = max([x; nodes.x]);
delta_x = max_x-min_x;
x_lim = round([min_x-0.05*delta_x max_x+0.05*delta_x]);
min_y = min([y; nodes.y]);
max_y = max([y; nodes.y]);
delta_y = max_y-min_y;
y_lim = round([min_y-0.05*delta_y max_y+0.05*delta_y]);

fig_counter = 1;
figure'

i = 2;
while i <= n
        while (nodes.id(i) >= nodes.id(i-1))
            nodes_x = [nodes_x; nodes.x(i)];
            nodes_y = [nodes_y; nodes.y(i)];
            disp(nodes.id(i))
            i = i + 1;
            node_time_stamp = nodes.stamp_sec(i-1) % Time stemp of last map publish
            if (i > n)
                break
            end
        end
            [x, y] = ground_truth_cutting(node_time_stamp, GT_table);
            plot(x,y,'LineWidth',1.5,'Color','b','LineStyle','-')
            xlim(x_lim); ylim(y_lim);
            sz = 75; % Scatter marke size
            hold on
            plot(nodes_x,nodes_y,'LineWidth',1.5,'Color','r')
            scatter(nodes_x(1),nodes_y(1),'red','filled','Marker','o','SizeData',sz)
            scatter(x(1),y(1),'blue','filled','Marker','o','SizeData',sz)
            scatter(x(end),y(end),'blue','diamond','filled','SizeData',sz)
            scatter(nodes_x(end),nodes_y(end),'red','diamond','filled','SizeData',sz)
            hold off
            title('Experience Map','FontSize',12,'Interpreter','latex')
            xlabel('$x$ (m)','FontSize',12,'Interpreter','latex');
            ylabel('$y$ (m)','FontSize',12,'Interpreter','latex');
            legend('ground truth','trajectory','Interpreter','latex','Location','best')            
            grid on
            hold off
            drawnow
      
            i = i + 1;

            if (i <= n) % Verify if doesn't exceed the array length
                  nodes_x = [nodes.x(i)];
                  nodes_y = [nodes.y(i)];
            end
            disp(i)
            if save_video
                writeVideo(vidObj, getframe(gcf));
            end
            if save_figures
                figure_name = "Figures/Exp_Map/map_evolution/PartialMap_"+num2str(fig_counter);
                print('-dpng', '-r600', figure_name+'.png');
                print('-depsc2', '-r600', figure_name+'.eps');
                fig_counter = fig_counter+1;
            end
end

%% Offset Correction
off_setx = nodes_x(1);
off_sety = nodes_y(1);

nodes_x = nodes_x - off_setx;
nodes_y = nodes_y - off_sety;

plot(x,y,'LineWidth',1.5,'Color','b','LineStyle','-')
xlim(x_lim); ylim(y_lim);
sz = 75; % Scatter marke size
hold on
plot(nodes_x,nodes_y,'LineWidth',1.5,'Color','r')
scatter(nodes_x(1),nodes_y(1),'red','filled','Marker','o','SizeData',sz)
scatter(x(1),y(1),'blue','filled','Marker','o','SizeData',sz)
scatter(x(end),y(end),'blue','diamond','filled','SizeData',sz)
scatter(nodes_x(end),nodes_y(end),'red','diamond','filled','SizeData',sz)
hold off
title('Experience Map - offset correction','FontSize',12,'Interpreter','latex')
xlabel('$x$ (m)','FontSize',12,'Interpreter','latex');
ylabel('$y$ (m)','FontSize',12,'Interpreter','latex');
legend('ground truth','trajectory','Interpreter','latex','Location','best')            
grid on
hold off
drawnow

if save_video
    writeVideo(vidObj, getframe(gcf));
end
if save_figures
    figure_name = "Figures/Exp_Map/map_evolution/PartialMap_"+num2str(fig_counter);
    print('-dpng', '-r600', figure_name+'.png');
    print('-depsc2', '-r600', figure_name+'.eps');
    fig_counter = fig_counter+1;
end


if save_video
    close(vidObj);
    % Convert to mp4: Need ffmeg in PATH
    system('ffmpeg -i Figures/Exp_Map/experience_map_evolution.avi -vf "scale=570:414" -c:v libx264 Figures/Exp_Map/experience_map_evolution.mp4');
end

print('-dpng', '-r600', 'Figures/Exp_Map/Final_Exp_map.png');
print('-depsc2', '-r600', 'Figures/Exp_Map/Final_Exp_map.eps');


%% Error Computation
% Make a resample of ground truth arrays to equalize the size with exp map
% arrays
new_len = length(nodes_y);

% Find max and minimum latitude values
[~, idx_max] = max(x);
[~, idx_min] = min(x);

% Ensures that extremes are included
indices_extremos = unique([1, idx_min, idx_max, length(x)]);

% Create a new vector of distributed indexes, including extremes
indices_reamostrados = round(linspace(1, length(x), new_len));
indices_finais = unique([indices_extremos, indices_reamostrados]);

% Reamostra o vetor mantendo os extremos
x_resampled = x(indices_finais(1:new_len));  % Adjust the length to new_len

% longitude vector
[~, idx_max] = max(y);
[~, idx_min] = min(y);
indices_reamostrados = round(linspace(1, length(y), new_len));
indices_extremos = unique([1, idx_min, idx_max, length(x)]);
indices_finais = unique([indices_extremos, indices_reamostrados]);

y_resampled = y(indices_finais(1:new_len));  % Adjust the length to new_len

erro_x = x_resampled - nodes_x;
erro_y = y_resampled - nodes_y;

erro_dist = zeros(new_len,1);
erro_sqrt = zeros(new_len,1);
for i=1:new_len
    erro_dist(i) = sqrt(erro_x(i)^2 + erro_y(i)^2);
    % erro_sqrt(i) = erro_x(i)^2 + erro_y(i)^2;
end
% erro_dist is always positive. It is the Euclidian distance between two
% points of both trajectories

% RMSe = sqrt(sum(erro_sqrt)/new_len)
erro_med = sum(erro_dist)/new_len;
k = 0:new_len-1;

figure'

plot(k,erro_dist,'LineWidth',1.5,'Color','b','LineStyle','-')
hold on
plot(k,erro_med*ones(new_len,1),'LineWidth',1.5,'Color','r','LineStyle','--')
xlim([0 new_len-1])
grid on
hold off
title(['Trajectory error - Average = ' num2str(erro_med) ' m'],'FontSize',12,'Interpreter','latex')
xlabel('$k$ (samples)','FontSize',12,'Interpreter','latex');
ylabel('Euclidian distance','FontSize',12,'Interpreter','latex');
legend('Euclidian distance','Average error','Interpreter','latex','Location','best') 

print('-dpng', '-r600', 'Figures/Exp_Map/Distance_error.png');
print('-depsc2', '-r600', 'Figures/Exp_Map/Distance_error.eps');

% Export to .csv file
headers = {'stamp_sec', 'Latitude', 'Longitude','x_estimated','y_estimated'};
stamp_sec = 0:1:new_len-1;
data = [stamp_sec' x_resampled y_resampled nodes_x nodes_y];

%%
function [x, y] = ground_truth_cutting(last_node_time_stamp, GT_table)

% % find time_stamp in GT_table
% [found, position] = ismember(last_node_time_stamp, GT_table.stamp_sec);
% if found
%     disp(position)
%     linha_encontrada = GT_table(position, :)
% end

i = 1;
while GT_table.stamp_sec(i) < last_node_time_stamp
    position = i;
    i = i + 1;
end

% cut gps table to last map time stamp
GT_table = GT_table(1:position-1, :);

% extract lat and long
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

end