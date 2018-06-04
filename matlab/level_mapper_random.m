clc
clear
close all

% load camera
% cam = CentralCamera('default');
p1 = zeros(10000,2);
p2 = zeros(10000,2);
p3 = zeros(10000,2);
p4 = zeros(10000,2);

p1_level = zeros(10000,2);
p2_level = zeros(10000,2);
p3_level = zeros(10000,2);
p4_level = zeros(10000,2);

count = 0;

width = 1280;
height = 1024;
fl = 0.01;  % focal length (m)
pixel_size = 10e-6; % (meters)
fl_pix = fl*(1/pixel_size);

cam = CentralCamera('focal', fl, 'pixel', pixel_size, ...
'resolution', [width height], 'centre', [width/2 height/2], 'name', 'mycamera');

cam.T = transl(0, 0, 2) * trotx(pi);

P = mkcube(0.5, 'pose', transl([0, 0, 0]) );    % make a cube of points
P = P(:,1:4);   % just a square

pixels = cam.project(P);
x_pix = pixels(1,:)' - width/2;
y_pix = pixels(2,:)' - height/2;

figure(1), clf
subplot(1,2,1)
cam.plot_camera()
plot_sphere(P, 0.03, 'r');  % display the cube
title('Downward-Facing Camera', 'Interpreter', 'latex')
xlabel('$x$', 'Interpreter', 'latex')
ylabel('$y$', 'Interpreter', 'latex')
zlabel('$z$', 'Interpreter', 'latex')
view(45,40)
grid on

subplot(1,2,2)
h1 = plot(x_pix, y_pix, 'r*');
hold on
h2 = plot(x_pix, y_pix, 'b*');
xlabel('$u$', 'Interpreter', 'latex')
ylabel('$v$', 'Interpreter', 'latex')
title('Image Plane (pixels)', 'Interpreter', 'latex')
legend({'$p$', '$p^*$'}, 'Interpreter', 'latex')
axis ij
axis equal
axis([-width/2, width/2, -height/2, height/2])
grid on

% pause;

phi_max = deg2rad(20);
theta_max = deg2rad(20);

% 10 seconds
t = 0:0.1:10;

% for i=1:length(t)
for i=1:1
    count = count + 1;
    % swing the camera around
    % pick random roll and pitch angles
    % phi = -phi_max + (phi_max - -phi_max) .* rand(1,1);
    % theta = -theta_max + (theta_max - -theta_max) .* rand(1,1);
    phi = deg2rad(9);
    theta = deg2rad(14);
    
    cam.T = transl(0, 0, 2) * trotx(pi + phi) * troty(theta);
    
    T_cam = cam.T.T;
    R_cam_vlc = T_cam(1:3,1:3);
    
    % get the projection
    pix = cam.project(P);
    
    % make pixel locations wrt to image center
    pix(1,:) = pix(1,:) - width/2;
    pix(2,:) = pix(2,:) - height/2;
    
    pix = pix';
    
    uvf = vertcat(pix', [fl_pix fl_pix fl_pix fl_pix], ones(1,4));
    
    % compose eq(14)
    hom = T_cam * uvf;
    
    u1 = fl_pix * hom(1,1)/hom(3,1);
    v1 = fl_pix * hom(2,1)/hom(3,1);
    
    u2 = fl_pix * hom(1,2)/hom(3,2);
    v2 = fl_pix * hom(2,2)/hom(3,2);
    
    u3 = fl_pix * hom(1,3)/hom(3,3);
    v3 = fl_pix * hom(2,3)/hom(3,3);
    
    u4 = fl_pix * hom(1,4)/hom(3,4);
    v4 = fl_pix * hom(2,4)/hom(3,4);
    
    x_pix_level = [u1, u2, u3, u4]';
    y_pix_level = [v1, v2, v3, v4]';
    
    x_pix = pix(:,1);
    y_pix = pix(:,2);
    
    % update plot handles
    h1.XData = x_pix;
    h1.YData = y_pix;
    
    h2.XData = x_pix_level;
    h2.YData = y_pix_level;
    
%     % store pixels for plotting
%     p1(count,1:2) = [x_pix(1), y_pix(1)];
%     p2(count,1:2) = [x_pix(2), y_pix(2)];
%     p3(count,1:2) = [x_pix(3), y_pix(3)];
%     p4(count,1:2) = [x_pix(4), y_pix(4)];
%     
%     p1_level(count,1:2) = [x_pix_level(1), y_pix_level(1)];
%     p2_level(count,1:2) = [x_pix_level(2), y_pix_level(2)];
%     p3_level(count,1:2) = [x_pix_level(3), y_pix_level(3)];
%     p4_level(count,1:2) = [x_pix_level(4), y_pix_level(4)];
    
    cam.plot_camera()
    pause(0.1)
end

% % trim
% p1 = p1(1:count,:);
% p2 = p2(1:count,:);
% p3 = p3(1:count,:);
% p4 = p4(1:count,:);
% 
% p1_level = p1_level(1:count,:);
% p2_level = p2_level(1:count,:);
% p3_level = p3_level(1:count,:);
% p4_level = p4_level(1:count,:);
% 
% % plot
% figure(2), clf
% plot(p1(:,1), p1(:,2), '*')
% hold on
% plot(p2(:,1), p2(:,2), '*')
% plot(p3(:,1), p3(:,2), '*')
% plot(p4(:,1), p4(:,2), '*')
% axis ij







