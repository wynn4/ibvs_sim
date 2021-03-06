clc
clear
close all

% ---------
% Plot Setup
% ---------
% Set Plot Width and FontSize
plot_width = 3.0;
aspect_ratio = 3/1; % 4:3 3:2 1:1
% Titel and Label Font size
font_size = 8.0;
% Legend font size
lfont_size = 8.0;
plot_height = plot_width * (1/aspect_ratio);
% Plot Line Widths
line_width = 1.1;
event_line_width = 0.5;
x_label_height = 0.17;

% load data which came from the following rosbag:
% ~/Desktop/rosbags/june_27/play test7n_2018-06-27-03-09-07.bag
load('june27_t7n_data.mat')

% estimates start after 47.5 seconds into the bag

t = ekf_vel_lpf(:,1);
t = t - t(1);
t = t + 47.5;

% auto mode engaged at t = 66 seconds
t = t(94:end,1);
vn = ekf_vel_lpf(94:end,2);
ve = ekf_vel_lpf(94:end,3);
t = t - t(1);

% Times when events took place starting the bag from t=66.0 seconds
t_wind_cal_start = 5.43;
t_wind_cal_stop = 15.43;
t_heading_cor = 19.45;
t_ibvs_outer = 23.5;
t_ibvs_inner = 42.6;
t_land_mode = 47.8;
t_land = 48.5;

t_end = 50;



figure('Units', 'inches', ...
       'Position', [0 0 plot_width plot_height], ...
       'PaperPositionMode', 'auto')
plot(t,vn,...
    'LineWidth', line_width)
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
  
% xlabel('Distance (m)',...
%        'Interpreter', 'latex',...
%        'FontUnits', 'points',...
%        'FontSize', font_size,...
%        'FontName', 'Times')
   
ylabel('$\dot{p}_n$ (m/s)',...
       'Interpreter', 'latex',...
       'FontUnits', 'points',...
       'FontSize', font_size,...
       'FontName', 'Times')

% legend({'Outer', 'Inner'},...
%        'Interpreter', 'latex',...
%        'FontUnits', 'points',...
%        'FontSize', lfont_size,...
%        'FontName', 'Times',...
%        'Location', 'East',...
%        'Orientation', 'vertical',...
%        'Box', 'on')

axis([0, t_end, -1, 1])
set(gca, ...
    'YTick', -1:0.5:1,...
    'XTick', 0:10:t_end,...
    'FontSize', font_size,...
    'FontName', 'Times')
grid on


figure('Units', 'inches', ...
       'Position', [0 0 plot_width plot_height+x_label_height], ...
       'PaperPositionMode', 'auto')
plot(t,ve,...
    'LineWidth', line_width)
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
   
ylabel('$\dot{p}_e$ (m/s)',...
       'Interpreter', 'latex',...
       'FontUnits', 'points',...
       'FontSize', font_size,...
       'FontName', 'Times')

% legend({'Outer', 'Inner'},...
%        'Interpreter', 'latex',...
%        'FontUnits', 'points',...
%        'FontSize', lfont_size,...
%        'FontName', 'Times',...
%        'Location', 'East',...
%        'Orientation', 'vertical',...
%        'Box', 'on')

axis([0, t_end, -2, 0])
set(gca, ...
    'YTick', -2:0.5:0,...
    'XTick', 0:10:t_end,...
    'FontSize', font_size,...
    'FontName', 'Times')
grid on

%EXPORT FIGURE
% -Select the figure window you want to export
% -in command window:
%  print -depsc2 path2plotfile.eps
%