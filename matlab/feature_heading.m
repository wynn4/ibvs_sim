clc
clear
close all

% load camera
% cam = CentralCamera('default');

heading_angle = deg2rad(15.0);

width = 1280;
height = 1024;
fl = 0.01;  % focal length (m)
pixel_size = 10e-6; % (meters)
fl_pix = fl*(1/pixel_size);

cam = CentralCamera('focal', fl, 'pixel', pixel_size, ...
'resolution', [width height], 'centre', [width/2 height/2], 'name', 'mycamera');

cam.T = transl(0, 0, 3) * trotx(pi) * trotz(heading_angle);

P = mkcube(0.5, 'pose', transl([0, 0, 0]) );    % make a cube of points
P = P(:,1:4);   % just a square

% re-order to be like aruco (P1 = TL, P2 = TR, P3 = BR, P4 = BL)
P1 = P(:,2);
P2 = P(:,3);
P3 = P(:,4);
P4 = P(:,1);

% re-define P
P = [P1, P2, P3, P4];

pixels = cam.project(P);
x_pix = pixels(1,:)' - width/2;
y_pix = pixels(2,:)' - height/2;

figure(1), clf
subplot(1,2,1)
cam.plot_camera()
plot_sphere(P1, 0.03, 'r');  % display the feature points
hold on
plot_sphere(P2, 0.03, 'g');
plot_sphere(P3, 0.03, 'b');
plot_sphere(P4, 0.03, 'k');
view(45,40)

subplot(1,2,2)
h1 = plot(x_pix(1), y_pix(1), 'r*');
hold on
h2 = plot(x_pix(2), y_pix(2), 'g*');
h3 = plot(x_pix(3), y_pix(3), 'b*');
h4 = plot(x_pix(4), y_pix(4), 'k*');

xlabel('u')
ylabel('v')
title('Image Plane (pixels)')
legend('p1', 'p2', 'p3', 'p4')
axis ij
axis equal
axis([-width/2, width/2, -height/2, height/2])

%
% compute heading angle
%

% feature pixels
p1 = [x_pix(1); y_pix(1)];
p2 = [x_pix(2); y_pix(2)];
p3 = [x_pix(3); y_pix(3)];
p4 = [x_pix(4); y_pix(4)];

% vector 1 is vector formed by p4 and p1

vec1 = p1 - p4
p1
p4


% define rotation from camera frame (level) to vehicle-1 frame (just the x and y components of each)
R_c_v1 = [0, -1; ...
          1, 0];
      
% rotate into vehicle-1 frame
vec1_v1 = R_c_v1*vec1;

heading = rad2deg(atan2(vec1_v1(2),vec1_v1(1)))























