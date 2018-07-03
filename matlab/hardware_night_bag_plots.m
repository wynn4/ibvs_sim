clc
clear
close all

% ---------
% Plot Setup
% ---------
% Set Plot Width and FontSize
plot_width = 3.0;
aspect_ratio = 4/3; % 4:3 3:2
% Titel and Label Font size
font_size = 8.0;
% Legend font size
lfont_size = 8.0;
plot_height = plot_width * (1/aspect_ratio);
%

% load copter's position data which came from the following rosbag:
% ~/Desktop/rosbags/june_11/test1nightlanding_2018-06-11-20-05-45.bag
load('pos_data_night.mat')

% get the data and trim off everything before we go into AUTO mode
t = mdata(5515:end,1);
pn = mdata(5515:end,2);
pe = mdata(5515:end,3);
pd = mdata(5515:end,4);

% fix time offset
t = t - t(1);

% Events (starting the bag at -s 93.5)
% 29.0 s: Enter IBVS mode
% 46.1 s: Switch to Inner Marker
% 50.7 s: Land mode engaged
% 51.4 s: Touchdown
time_shift = 0.0;
t_ibvs = 29.0 + time_shift;
t_switch = 46.1 + time_shift;
t_land = 50.7 + time_shift;
t_landed = 51.4 + time_shift;

% figure(1), clf
figure('Units', 'inches', ...
       'Position', [0 0 plot_width plot_height], ...
       'PaperPositionMode', 'auto')


subplot(3,1,1)
plot(t,pn)
hold on
% plot IBVS line
plot([t_ibvs; t_ibvs], [-100, 100], ':k')
% plot switch to inner marker line
plot([t_switch; t_switch], [-100, 100], '--k')
% plot land mode line
plot([t_land; t_land], [-100, 100], '-.k')
% plot touchdown line
plot([t_landed; t_landed], [-100, 100], '-k')

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
   
ylabel('$P_N (m)$',...
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

axis([0, 60, -5, 5])
set(gca, ...
    'YTick', -5:5:5,...
    'XTick', 0:10:60)
grid on

subplot(3,1,2)
plot(t,pe)
hold on
% plot IBVS line
plot([t_ibvs; t_ibvs], [-100, 100], ':k')
% plot switch to inner marker line
plot([t_switch; t_switch], [-100, 100], '--k')
% plot land mode line
plot([t_land; t_land], [-100, 100], '-.k')
% plot touchdown line
plot([t_landed; t_landed], [-100, 100], '-k')

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
   
ylabel('$P_E (m)$',...
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

axis([0, 60, -13, 0])
set(gca, ...
    'YTick', -10:5:0,...
    'XTick', 0:10:60)
grid on

subplot(3,1,3)
plot(t,pd)
hold on
% plot IBVS line
plot([t_ibvs; t_ibvs], [-100, 100], ':k')
% plot switch to inner marker line
plot([t_switch; t_switch], [-100, 100], '--k')
% plot land mode line
plot([t_land; t_land], [-100, 100], '-.k')
% plot touchdown line
plot([t_landed; t_landed], [-100, 100], '-k')

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
   
ylabel('$P_D (m)$',...
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

axis([0, 60, -15, 1])
set(gca, ...
    'YTick', -15:5:0,...
    'XTick', 0:10:60)
grid on


% print x0
pn(1)
pe(1)
pd(1)


%
% Attitude Figs
%


% load euler angle data which came from same bag as above
load('euler_data_night.mat')

% pull off the data
t = euler_data(5423:end,1);
phi = euler_data(5423:end,2);
theta = euler_data(5423:end,3);
psi = euler_data(5423:end,4);


% fix time offset
t = t - t(1);

% figure(1), clf
figure('Units', 'inches', ...
       'Position', [0 0 plot_width plot_height], ...
       'PaperPositionMode', 'auto')
   
subplot(3,1,1)
plot(t, phi)
subplot(3,1,2)
plot(t, theta)
subplot(3,1,3)
plot(t, psi)



%EXPORT FIGURE
% -Select the figure window you want to export
% -in command window:
%  print -depsc2 path2plotfile.eps
%
