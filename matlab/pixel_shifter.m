clc
clear

width = 1288;
height = 964;

% desired final pixel locations
p_des = [-500, 400;
         -400, 400;
         -400, 300;
         -500, 300];
     
cent_des = [sum(p_des(:,1))/4, sum(p_des(:,2))/4]

% starting pixel locations
pix = [100, 100;
       200, 100;
       200, 200;
       100, 200]; % [u, v]
   
x_pix = pix(:,1);
y_pix = pix(:,2);

figure(1), clf
h1 = plot(x_pix(1), y_pix(1), 'r*');
hold on
h2 = plot(x_pix(2), y_pix(2), 'g*');
h3 = plot(x_pix(3), y_pix(3), 'b*');
h4 = plot(x_pix(4), y_pix(4), 'k*');
xlabel('u')
ylabel('v')
title('Image Plane (pixels)')

axis ij
axis equal
axis([-width/2, width/2, -height/2, height/2])

% plot(p_des(:,1), p_des(:,2), '*')
legend('p1', 'p2', 'p3', 'p4', 'des pix')

% get the avg side length and center of the marker

l1 = norm(pix(1,:) - pix(2,:));  % p1 - p2
l2 = norm(pix(2,:) - pix(3,:));  % p2 - p3
l3 = norm(pix(3,:) - pix(4,:));  % p3 - p4
l4 = norm(pix(4,:) - pix(1,:));  % p4 - p1

side_length = (l1 + l2 + l3 + l4) / 4

center = [sum(pix(:,1))/4, sum(pix(:,2))/4]

plot(center(1), center(2), '*')

% get the direciton vector from where the marker first appears to where the
% marker needs to end up
vec = cent_des - center;

% get the distance in pixels that we need to travel
len = norm(vec);

% normalize the dir vector
vec = vec/norm(vec);

% split up into 4 legs
len_each = len / 4

%
% start moving
%

c = center + vec * len_each;
[new_corners, new_center] = make_square(c, side_length);

pause(1.0)

h1.XData = new_corners(1,1);
h1.YData = new_corners(1,2);
h2.XData = new_corners(2,1);
h2.YData = new_corners(2,2);
h3.XData = new_corners(3,1);
h3.YData = new_corners(3,2);
h4.XData = new_corners(4,1);
h4.YData = new_corners(4,2);

c = new_center + vec * len_each;

[new_corners, new_center] = make_square(c, side_length);

pause(1.0)

h1.XData = new_corners(1,1);
h1.YData = new_corners(1,2);
h2.XData = new_corners(2,1);
h2.YData = new_corners(2,2);
h3.XData = new_corners(3,1);
h3.YData = new_corners(3,2);
h4.XData = new_corners(4,1);
h4.YData = new_corners(4,2);

c = new_center + vec * len_each;

[new_corners, new_center] = make_square(c, side_length);

pause(1.0)

h1.XData = new_corners(1,1);
h1.YData = new_corners(1,2);
h2.XData = new_corners(2,1);
h2.YData = new_corners(2,2);
h3.XData = new_corners(3,1);
h3.YData = new_corners(3,2);
h4.XData = new_corners(4,1);
h4.YData = new_corners(4,2);

c = new_center + vec * len_each;

[new_corners, new_center] = make_square(c, side_length);

pause(1.0)

h1.XData = new_corners(1,1);
h1.YData = new_corners(1,2);
h2.XData = new_corners(2,1);
h2.YData = new_corners(2,2);
h3.XData = new_corners(3,1);
h3.YData = new_corners(3,2);
h4.XData = new_corners(4,1);
h4.YData = new_corners(4,2);

function [corners, center] = make_square(center, side_length)

diag = sqrt(2) * side_length;
p1 = center' + [-sqrt(2)/2; -sqrt(2)/2] * (diag/2);
p2 = center' + [sqrt(2)/2; -sqrt(2)/2] * (diag/2);
p3 = center' + [sqrt(2)/2; sqrt(2)/2] * (diag/2);
p4 = center' + [-sqrt(2)/2; sqrt(2)/2] * (diag/2);

corners = [p1'; p2'; p3'; p4'];

center = zeros(1,2);
center(1) = sum(corners(:,1))/4;
center(2) = sum(corners(:,2))/4;
end

































