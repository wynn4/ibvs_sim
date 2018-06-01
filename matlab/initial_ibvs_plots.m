clc
clear
close all

% load copter's position data
load('initial_ibvs_test_copter_NED_data.mat');

% load the ibvs image error data
load('ibvs_initial_data.mat')

% cut off stuff before ibvs became active
arr = arr(6:end,:);

% adjust time to start at 0
t = arr(:,1);
t = t - t(1);

% pull off the error data
e1 = arr(:,2);
e2 = arr(:,3);
e3 = arr(:,4);
e4 = arr(:,5);

t_copter = mdata(:,1);
pn_copter = mdata(:,2);
pe_copter = mdata(:,3);
pd_copter = mdata(:,4);

figure(1), clf
plot(t,e1, t,e2, t,e3, t,e4)
axis([0, 25, 0, 550])
title('Pixel Error $(p_{des} - p^{*})$ vs Time', 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Error (pixels)', 'Interpreter', 'latex')
legend({'e1', 'e2', 'e3', 'e4'}, 'Interpreter', 'latex')
grid on

figure(2), clf
subplot(3,1,1)
plot(t_copter, pn_copter)
title('Multirotor and ArUco Position vs Time Under IBVS Control', 'Interpreter', 'latex')
hold on
plot([0; 30], [3; 3], '--')
axis([0, 25, 0, 4])
ylabel('$P_n$ (m)', 'Interpreter', 'latex')
legend({'Multirotor', 'ArUco Marker'}, 'Interpreter', 'latex', 'Location','SouthEast')
grid on

subplot(3,1,2)
plot(t_copter, pe_copter)
hold on
plot([0; 30], [3; 3], '--')
axis([0, 25, 0, 4])
ylabel('$P_e$ (m)', 'Interpreter', 'latex')
grid on

subplot(3,1,3)
plot(t_copter, pd_copter)
hold on
plot([0; 30], [0; 0], '--')
axis([0, 25, -10, 1])
ylabel('$P_d$ (m)', 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')
grid on



