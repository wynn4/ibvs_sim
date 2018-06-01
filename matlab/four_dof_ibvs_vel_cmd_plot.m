clc
clear
close all

% load copter's position data
load('4dof_ibvs_vel_cmd_data.mat');

t = data(:,1);

vx = data(:,2);
vy = data(:,3);
vz = data(:,4);

wx = data(:,5);
wy = data(:,6);
wz = data(:,7);

% plots
subplot(4,1,1)
plot(t, vx)
title('4DOF IBVS Control Output Velocities vs Time', 'Interpreter', 'latex')
axis([0, 25, -11 11])
ylabel('$v_x$ (m/s)', 'Interpreter', 'latex')
grid on

subplot(4,1,2)
plot(t, vy)
axis([0, 25, -11 11])
ylabel('$v_y$ (m/s)', 'Interpreter', 'latex')
grid on

subplot(4,1,3)
plot(t, vz)
axis([0, 25, -10 10])
ylabel('$v_z$ (m/s)', 'Interpreter', 'latex')
grid on

subplot(4,1,4)
plot(t, wz)
axis([0, 25, -0.2 0.2])
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('$\omega_z$ (rad/s)', 'Interpreter', 'latex')
grid on