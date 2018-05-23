clc
clear
close all

% 4 DOF IBVS Plot
load('ibvs4dof_data_outer.mat');

t = arr(:,1);
t = t - t(1);

e1 = arr(:,2);
e2 = arr(:,3);
e3 = arr(:,4);
e4 = arr(:,5);

figure(1), clf
plot(t,e1, t,e2, t,e3, t,e4)
axis([0, 30, 0, 275])
title('Pixel Error $(p_{des} - p^{*})$ vs Time', 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Error (pixels)', 'Interpreter', 'latex')
legend({'e1', 'e2', 'e3', 'e4'}, 'Interpreter', 'latex')

% 6 DOF IBVS Plot
load('ibvs6dof_data_outer.mat');

t = arr(:,1);
t = t - t(1);

e1 = arr(:,2);
e2 = arr(:,3);
e3 = arr(:,4);
e4 = arr(:,5);

figure(2), clf
plot(t,e1, t,e2, t,e3, t,e4)
axis([0, 30, 0, 275])
title('Pixel Error $(p_{des} - p^{*})$ vs Time', 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Error (pixels)', 'Interpreter', 'latex')
legend({'e1', 'e2', 'e3', 'e4'}, 'Interpreter', 'latex')


