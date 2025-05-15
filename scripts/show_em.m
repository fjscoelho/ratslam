%% Show Experience Map

clear
close all;
clc;

% Set to true to save a experience map evolution video
save_video = false;

% Set to true to save partial fiugres of map evolution
save_figures = false;

if save_video
    numFrames = 50; % Número de iterações/timesteps
    videoFile = 'Figures/Exp_Map/experience_map_evolution.avi';
    vidObj = VideoWriter(videoFile);
    vidObj.FrameRate = 5;
    vidObj.Quality = 100; 
    open(vidObj);
end

% Load experience map data
nodes = readtable('exported_data/nodes.csv');
links = readtable('exported_data/links.csv');

% Load Ground Truth
GT_table = readtable('exported_data/ground_truth.csv');

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

min_x = min(nodes.x);
max_x = max(nodes.x);
delta_x = max_x-min_x;
x_lim = round([min_x-0.05*delta_x max_x+0.05*delta_x])
min_y = min(nodes.y);
max_y = max(nodes.y);
delta_y = max_y-min_y;
y_lim = round([min_y-0.05*delta_y max_y+0.05*delta_y])

fig_counter = 1;
figure'
for i=1:n
    if i == 1
        id = 0;
    else
        if nodes.id(i) >= nodes.id(i-1)
            nodes_x = [nodes_x; nodes.x(i)];
            nodes_y = [nodes_y; nodes.y(i)];
            % disp(nodes.id(i))
        else
            plot(x,y,'LineWidth',1.5,'Color','b','LineStyle','-')
            xlim(x_lim); ylim(y_lim);
            sz = 75; % Scatter marke size
            hold on
            plot(nodes_x,nodes_y,'LineWidth',1.5,'Color','r')
            scatter(nodes_x(1),nodes_y(1),'blue','filled','Marker','o','SizeData',sz)
            scatter(x(end),y(end),'blue','diamond','filled','SizeData',sz)
            scatter(nodes_x(end),nodes_y(end),'red','diamond','filled','SizeData',sz)
            hold off
            title('Experience Map','FontSize',12,'Interpreter','latex')
            xlabel('$x$ (m)','FontSize',12,'Interpreter','latex');
            ylabel('$y$ (m)','FontSize',12,'Interpreter','latex');
            legend('ground truth','trajectory','Interpreter','latex','Location','best')            
            grid on
            nodes_x = [0];
            nodes_y = [0];
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
        end
    end
end

if save_video
    close(vidObj);
    % Convert to mp4: Need ffmeg in PATH
    system('ffmpeg -i Figures/Exp_Map/experience_map_evolution.avi -vf "scale=570:414" -c:v libx264 Figures/Exp_Map/experience_map_evolution.mp4');
end

print('-dpng', '-r600', 'Figures/Exp_Map/Final_Exp_map.png');
print('-depsc2', '-r600', 'Figures/Exp_Map/Final_Exp_map.eps');