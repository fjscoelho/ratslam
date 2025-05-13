% showvt.m
% 
% .m file to load and visualize visual template and experience map histories
% for a dataset, from logfiles created by OpenRatSLAM
% 
% Copyright (C) 2012
% Michael Milford (michael.milford@qut.edu.au)
% 
% 1. Queensland University of Technology, Australia
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.


% Plot a template graph

clear
close all;
clc;
figh = figure;

vt = csvread('vt_id.csv', 1, 0);
vt_length = length(vt);
start = vt(1, 1);
t1 = vt(1:vt_length, 1) - start;

vt_id = vt(1:vt_length, 3);
plot(t1, vt_id, 'rx',"LineWidth",1.1);
grid on

em = csvread('em_id.csv', 1, 0);
em_length = length(em);
t2 = em(1:em_length, 1) - start;
dest_id = em(1:em_length, 5);

hold on;
plot(t2, dest_id, 'bo','LineWidth',1.1);
hold off;

xlabel('$t$ (s)','FontSize',12,'Interpreter','latex');
ylabel('Template / Experience ID Number', 'FontSize',12,'Interpreter','latex');

% axis tight;
legend('Template ID', 'Experience ID', 'Location', 'NorthWest','Interpreter','latex','Fontsize',12);

print -r300 -djpeg100 Figures/vt_em_ids.png
print('-depsc2', '-r600', 'Figures/vt_em_ids.eps');