clc
clear
close all

% Load mat files
data_outer = load('outer_data_nested_sim.mat');
data_inner = load('inner_data_nested_sim.mat');

% Trim off all of the data before and after IBVS mode was active
ar = data_outer.arr(:,32);
idx = find(ar);
first = idx(1);
last = idx(end);
data_outer = data_outer.arr(first:last,:);

ar = data_inner.arr(:,32);
idx = find(ar);
first = idx(1);
last = idx(end);
data_inner = data_inner.arr(first:last,:);

% Now we want to combine the two sets of data and split them where the
% switchover occurs

% get everything before switch to inner marker
idx = find(data_outer(:,33), 1);
idx = idx - 1;
data_outer = data_outer(1:idx,:);

% get everything after switch to inner marker
idx = find(data_inner(:,33), 1);
data_inner = data_inner(idx:end,:);

% combine them
data = [data_outer; data_inner];

% now get the data we care about
t = data(:,1);
t = t - t(1);

idx = find(data(:,33), 1);
t_switch = t(idx);
t_switch = [t_switch; t_switch];
line = [0; 300];

e1 = data(:,2);
e2 = data(:,3);
e3 = data(:,4);
e4 = data(:,5);

figure(1), clf
plot(t,e1, t,e2, t,e3, t,e4)
% axis([0, 30, 0, 275])
hold on
title('Pixel Error $(p_{des} - p^{*})$ vs Time', 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Error (pixels)', 'Interpreter', 'latex')
plot(t_switch, line, '--k')
legend({'e1', 'e2', 'e3', 'e4', 'Switch to Inner Marker'}, 'Interpreter', 'latex')

