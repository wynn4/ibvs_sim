clc
clear
close all

% load copter's position data
load('test1_positioningibvs_data_outer.mat')

load('hardware_ibvs_pos_data.mat')

% cut off stuff before ibvs became active
arr = arr(137:end,:);

% adjust time to start at 0
t = arr(:,1);
t = t - t(1);

% pull off the error data
e1 = arr(:,2);
e2 = arr(:,3);
e3 = arr(:,4);
e4 = arr(:,5);

t_copter = hw_data(1961:4062,1);
t_copter = t_copter - t_copter(1);
pn_copter = hw_data(1961:4062,2);
pe_copter = hw_data(1961:4062,3);
pe_copter = pe_copter - pe_copter(1);
pd_copter = hw_data(1961:4062,4);

figure(1), clf
plot(t,e1, t,e2, t,e3, t,e4)
axis([0, 30, 0, 200])
title('Pixel Error $(p_{des} - p^{*})$ vs Time (Hardware Test)', 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Error (pixels)', 'Interpreter', 'latex')
legend({'e1', 'e2', 'e3', 'e4'}, 'Interpreter', 'latex')
grid on

figure(2), clf
subplot(3,1,1)
plot(t_copter, pn_copter)
title('Multirotor and ArUco Position vs Time (Hardware Test)', 'Interpreter', 'latex')
hold on
plot([0; 30], [0; 0], '--')
axis([0, 30, -1, 1])
ylabel('$P_n$ (m)', 'Interpreter', 'latex')
legend({'Multirotor', 'ArUco Marker'}, 'Interpreter', 'latex', 'Location','NorthEast')
grid on

subplot(3,1,2)
plot(t_copter, pe_copter)
hold on
plot([0; 30], [0; 0], '--')
axis([0, 30, -1, 1])
ylabel('$P_e$ (m)', 'Interpreter', 'latex')
grid on

subplot(3,1,3)
plot(t_copter, pd_copter)
hold on
plot([0; 30], [0; 0], '--')
axis([0, 30, -7, 1])
ylabel('$P_d$ (m)', 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')
grid on