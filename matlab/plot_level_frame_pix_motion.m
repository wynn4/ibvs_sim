clc
clear
close all

load('ibvs_data_outer.mat')
im_width = 962;
im_height = 720;

% time
t = arr(:,1);
t = t - t(1);

% true pixel locations
p1_x = arr(:,6) - im_width/2;
p1_y = arr(:,7) - im_height/2;

% level mapped pixel locations
p1_level_x = arr(:,14);
p1_level_y = arr(:,15);


% roll and pitch angles
roll = arr(:,30);
pitch = arr(:,31);

% trim off data you don't want
start_index = 1;
end_index = length(t);

t = t(start_index:end_index);
p1_x = p1_x(start_index:end_index);
p1_y = p1_y(start_index:end_index);
p1_level_x = p1_level_x(start_index:end_index);
p1_level_y = p1_level_y(start_index:end_index);
roll = roll(start_index:end_index);
pitch = pitch(start_index:end_index);

figure(1), clf
subplot(2,1,1)
plot(t,p1_x, t, p1_level_x,'-.')
% plot(t,p1_x, t, p1_level_x,'-.', 'LineWidth', 2)
axis([0, 30, -400, 400])
title('Feature $p_u$ Location vs Time', 'Interpreter', 'latex')
ylabel('$u$ (pixels)', 'Interpreter', 'latex')
legend({'$p_u$', '$p_u^*$'}, 'Interpreter', 'latex')

subplot(2,1,2)
plot(t,p1_y, t, p1_level_y,'-.')
axis([0, 30, -400, 100])
title('Feature $p_v$ Location vs Time', 'Interpreter', 'latex')
ylabel('$v$ (pixels)', 'Interpreter', 'latex')
legend({'$p_v$', '$p_v^*$'}, 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

% subplot(3,1,3)
% plot(t,roll, t,pitch)
% title('Roll and Pitch Angles vs Time')
% xlabel('Time (s)')
% ylabel('Attitude Angle (rad)')
% legend('roll', 'pitch')

% title('Feature Pixel u Location vs time')
% title('Feature Pixel v Location vs time')
% title('Roll and Pitch Angles vs Time')



