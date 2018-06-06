clc
clear
close all

% load copter's position data
load('hardware_ibvs_vel_cmd_data.mat');

t = vel_data(:,1);

vx = vel_data(:,2);
vy = vel_data(:,3);
vz = vel_data(:,4);

wx = vel_data(:,5);
wy = vel_data(:,6);
wz = vel_data(:,7);

% plots
subplot(4,1,1)
plot(t, vx)
title('IBVS Control Output Velocities vs Time (Hardware Test)', 'Interpreter', 'latex')
axis([0, 30, -1 1])
ylabel('$v_x$ (m/s)', 'Interpreter', 'latex')
grid on

subplot(4,1,2)
plot(t, vy)
axis([0, 30, -1 1])
ylabel('$v_y$ (m/s)', 'Interpreter', 'latex')
grid on

subplot(4,1,3)
plot(t, vz)
axis([0, 30, -5 10])
ylabel('$v_z$ (m/s)', 'Interpreter', 'latex')
grid on

subplot(4,1,4)
plot(t, wz)
axis([0, 30, -0.5 0.5])
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('$\omega_z$ (rad/s)', 'Interpreter', 'latex')
grid on