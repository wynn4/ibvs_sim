clc
clear
close all

% ---------
% Plot Setup
% ---------
% Set Plot Width and FontSize
plot_width = 3.0;
aspect_ratio = 4/3; % 4:3 3:2 1:1
% Titel and Label Font size
font_size = 8.0;
% Legend font size
lfont_size = 6.0;
plot_height = plot_width * (1/aspect_ratio);
% Plot Line Widths
line_width = 1.1;
event_line_width = 0.5;
x_label_height = 0.17;

% load aruco distance data which came from the following rosbag:
% ~/Desktop/rosbags/june_11/test1nightlanding_2018-06-11-20-05-45.bag
load('aruco_dist_data.mat');

t_outer = outer(:,1);
d_outer = outer(:,2);

t_outer = t_outer - t_outer(1);

% first detection occurs 2.54 seconds after the auto mode is engaged at
% 93.5 seconds in the rosbag
t_outer = t_outer + 2.54;

t_inner = inner(:,1);
d_inner = inner(:,2);
t_inner = t_inner - t_inner(1);

% first inner detection occurs 42.522 seconds after the auto mode is engaged at
% 93.5 seconds in the rosbag
t_inner = t_inner + 42.522;

figure('Units', 'inches', ...
       'Position', [0 0 plot_width plot_height], ...
       'PaperPositionMode', 'auto')
   
plot(t_outer, d_outer, '--', 'LineWidth', line_width)
hold on
plot(t_inner, d_inner, '-r', 'LineWidth', line_width)
% title('Marker Detection Rate vs Distance from Marker',...
%       'Interpreter', 'latex',...
%       'FontUnits', 'points',...
%       'FontSize', font_size,...
%       'FontName', 'Times')
  
xlabel('Time (s)',...
       'Interpreter', 'latex',...
       'FontUnits', 'points',...
       'FontSize', font_size,...
       'FontName', 'Times')
   
ylabel('ArUco Distance (m)',...
       'Interpreter', 'latex',...
       'FontUnits', 'points',...
       'FontSize', font_size,...
       'FontName', 'Times')

legend({'Outer', 'Inner'},...
       'Interpreter', 'latex',...
       'FontUnits', 'points',...
       'FontSize', lfont_size,...
       'FontName', 'Times',...
       'Location', 'NorthEast',...
       'Orientation', 'vertical',...
       'Box', 'on')

axis([0, 60, 0, 16])
set(gca, ...
    'YTick', 0:5:15,...
    'XTick', 0:10:60,...
    'FontSize', font_size,...
    'FontName', 'Times')
grid on

%EXPORT FIGURE
% -Select the figure window you want to export
% -in command window:
%  print -depsc2 path2plotfile.eps
%