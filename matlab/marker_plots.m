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



dist = [0, 0.125, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 7.0, ...
        8.0, 9.0, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]';
    
outer_rate = [0, 0, 0, 0, 29.8, 28.5, 29.8, 30, 29.9, 30, 30, 30.1, 30, 29.9, ...
              29.8, 29.0, 28.5, 29, 30, 28.9, 30, 29.0, 22.0, 6.0, 0.1, 0]';
          
inner_rate = [0, 25, 30, 29.5, 30, 31.0, 29.8, 30.0, 29.3, 20.9, 5.0, 1.0, 0.1, 0.0, ...
              0, 0, 0 , 0, 0, 0, 0, 0, 0 ,0, 0, 0]';
          
% figure(1), clf
figure('Units', 'inches', ...
       'Position', [0 0 plot_width plot_height], ...
       'PaperPositionMode', 'auto')

plot(dist, outer_rate, 'o-', 'LineWidth', line_width)
hold on
plot(dist, inner_rate, 's-', 'LineWidth', line_width)

% title('Marker Detection Rate vs Distance from Marker',...
%       'Interpreter', 'latex',...
%       'FontUnits', 'points',...
%       'FontSize', font_size,...
%       'FontName', 'Times')
  
xlabel('Distance (m)',...
       'Interpreter', 'latex',...
       'FontUnits', 'points',...
       'FontSize', font_size,...
       'FontName', 'Times')
   
ylabel('Detection Rate (Hz)',...
       'Interpreter', 'latex',...
       'FontUnits', 'points',...
       'FontSize', font_size,...
       'FontName', 'Times')

legend({'Outer', 'Inner'},...
       'Interpreter', 'latex',...
       'FontUnits', 'points',...
       'FontSize', lfont_size,...
       'FontName', 'Times',...
       'Location', 'East',...
       'Orientation', 'vertical',...
       'Box', 'on')

axis([0, 20, -5, 35])
set(gca, ...
    'YTick', -5:5:35,...
    'XTick', 0:2.5:20,...
    'FontSize', font_size,...
    'FontName', 'Times')
grid on

figure(2), clf
plot(dist, outer_rate, 'o-', dist, inner_rate, 's-')
title('Marker Detection Rate vs Distance from Marker', 'Interpreter', 'latex')
xlabel('Distance (m)', 'Interpreter', 'latex')
ylabel('Detection Rate (Hz)', 'Interpreter', 'latex')
legend({'Outer Marker', 'Inner Marker'}, 'Interpreter', 'latex')
axis([0, 4, -1, 35])
grid on

% figure(3), clf
% plot(dist, outer_rate, 'o-', dist, inner_rate, 's-')
% title('Marker Detection Rate vs Distance from Marker', 'Interpreter', 'latex')
% xlabel('Distance (m)', 'Interpreter', 'latex')
% ylabel('Detection Rate (Hz)', 'Interpreter', 'latex')
% legend({'Outer Marker', 'Inner Marker'}, 'Interpreter', 'latex')
% axis([0, 1, -1, 35])
% grid on

%EXPORT FIGURE
% -Select the figure window you want to export
% -in command window:
%  print -depsc2 path2plotfile.eps
%

