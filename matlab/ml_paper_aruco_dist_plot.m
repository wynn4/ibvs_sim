clc
clear
close all

% ---------
% Plot Setup
% ---------
% Set Plot Width and FontSize
plot_width = 3.0;
aspect_ratio = 4/2.5; % 4:3 3:2 1:1
% Titel and Label Font size
font_size = 8.0;
% Legend font size
lfont_size = 6.0;
plot_height = plot_width * (1/aspect_ratio);
% Plot Line Widths
line_width = 1.1;
event_line_width = 0.5;


% load aruco distance data which came from the following rosbag:
% ~/Desktop/rosbags/june_27/test7n_2018-06-27-03-09-07.bag
load('ml_aruco_dist_data.mat');

t_outer = outer(:,1);
d_outer = outer(:,2);

t_outer = t_outer - t_outer(1);

% first detection occurs 4.35 seconds after the auto mode is engaged at
% 66.0 seconds into the rosbag
t_outer = t_outer + 4.35;

t_inner = inner(1:end-1,1);
d_inner = inner(1:end-1,2);
t_inner = t_inner - t_inner(1);

% first inner detection occurs 39.01 seconds after the auto mode is engaged at
% 66.0 seconds into the rosbag
t_inner = t_inner + 39.01;

% Times when events took place starting the bag from t=66.0 seconds
t_wind_cal_start = 5.43;
t_wind_cal_stop = 15.43;
t_heading_cor = 19.45;
t_ibvs_outer = 23.5;
t_ibvs_inner = 42.6;
t_land_mode = 47.8;
t_land = 48.5;

figure('Units', 'inches', ...
       'Position', [0 0 plot_width plot_height], ...
       'PaperPositionMode', 'auto')

matlab_blue = [0, 0.4470, 0.7410];
plot(t_outer, d_outer, '--', 'LineWidth', line_width)
hold on
plot(t_inner, d_inner, '-r', 'LineWidth', line_width)
hold on
plot([t_wind_cal_start; t_wind_cal_start], [-100; 100], '-.k', 'LineWidth', event_line_width)
plot([t_wind_cal_stop; t_wind_cal_stop], [-100; 100], '-.k', 'LineWidth', event_line_width)
plot([t_heading_cor; t_heading_cor], [-100; 100], '-.k', 'LineWidth', event_line_width)
plot([t_ibvs_outer; t_ibvs_outer], [-100; 100], '-.k', 'LineWidth', event_line_width)
plot([t_ibvs_inner; t_ibvs_inner], [-100; 100], '-.k', 'LineWidth', event_line_width)
plot([t_land_mode; t_land_mode], [-100; 100], '-.k', 'LineWidth', event_line_width)
plot([t_land; t_land], [-100; 100], '-.k', 'LineWidth', event_line_width)
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

axis([0, 50, 0, 17])
set(gca, ...
    'YTick', 0:5:15,...
    'XTick', 0:10:60,...
    'FontSize', font_size,...
    'FontName', 'Times')
grid on

% Get default colors
% get(gca,'colororder')

%EXPORT FIGURE
% -Select the figure window you want to export
% -in command window:
%  print -depsc2 path2plotfile.eps
%