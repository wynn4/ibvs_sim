clc
clear
close all

% Load pixel error mat files
data_outer = load('outer_data_truth_landing_sim.mat');
data_inner = load('inner_data_truth_landing_sim.mat');
% data_outer = load('/home/jesse/ibvs_data_outer.mat');
% data_inner = load('/home/jesse/ibvs_data_inner.mat');

% Load UAS position data
load('truth_landing_pos_data.mat');

% Trim off all of the data before and after IBVS mode was active
ar = data_outer.arr(:,32);
idx = find(ar);
first = idx(1);
last = idx(end);
data_outer = data_outer.arr(first:last,:);

t_ibvs = data_outer(1,1);

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
%t = t - t(1);

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
axis([12, 45, 0, 275])
hold on
title('Pixel Error $(\mathbf{p}_{des} - \mathbf{p}^{*})$ vs Time During IBVS Landing', 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Error (pixels)', 'Interpreter', 'latex')
plot(t_switch, line, '--k')
legend({'e1', 'e2', 'e3', 'e4', 'Switch to Inner Marker'}, 'Interpreter', 'latex')
grid on


t = mdata(:,1);
pn = mdata(:,2);
pe = mdata(:,3);
pd = mdata(:,4);

t_land = 39.7;

% position plot
figure(2), clf
subplot(3,1,1)
plot(t, pn)
hold on
plot([0; 45], [9; 9], '--')
plot([t_ibvs; t_ibvs], [0; 10], '-.k')
plot([t_land; t_land], [0; 10], ':k')
title('Multirotor and ArUco Position vs Time During IBVS Landing', 'Interpreter', 'latex')
legend({'Multirotor', 'ArUco Marker', 'IBVS Active', 'Landing'}, 'Interpreter', 'latex', 'Location','SouthEast')
axis([0 45 0 10])
ylabel('$P_n$ (m)', 'Interpreter', 'latex')
grid on

subplot(3,1,2)
plot(t, pe)
hold on
plot([0; 45], [9; 9], '--')
plot([t_ibvs; t_ibvs], [0; 10], '--k')
plot([t_land; t_land], [0; 10], ':k')
axis([0 45 0 10])
ylabel('$P_e$ (m)', 'Interpreter', 'latex')
grid on

subplot(3,1,3)
plot(t, pd)
hold on
plot([0; 45], [0; 0], '--')
plot([t_ibvs; t_ibvs], [-11; 1], '--k')
plot([t_land; t_land], [-11; 1], ':k')
axis([0 45 -11 1])
ylabel('$P_d$ (m)', 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')
grid on


