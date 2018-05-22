clc
clear
close all

load('/home/jesse/ibvs_data_outer.mat')
im_width = 962;
im_height = 720;

% time
t = arr(:,1);

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
subplot(3,1,1)
plot(t,p1_x, t, p1_level_x)
title('Feature Pixel u Location vs time')
ylabel('u (pixels)')
legend('true pixel', 'level-frame pixel')

subplot(3,1,2)
plot(t,p1_y, t, p1_level_y)
title('Feature Pixel v Location vs time')
ylabel('v (pixels)')
legend('true pixel', 'level-frame pixel')

subplot(3,1,3)
plot(t,roll, t,pitch)
title('Roll and Pitch Angles vs Time')
xlabel('Time (s)')
ylabel('Attitude Angle (rad)')
legend('roll', 'pitch')

% title('Feature Pixel u Location vs time')
% title('Feature Pixel v Location vs time')
% title('Roll and Pitch Angles vs Time')



