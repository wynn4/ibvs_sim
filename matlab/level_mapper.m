clc
clear
close all

% load camera
% cam = CentralCamera('default');

width = 1280;
height = 1024;
fl = 0.01;  % focal length (m)
pixel_size = 10e-6; % (meters)
fl_pix = fl*(1/pixel_size);

cam = CentralCamera('focal', fl, 'pixel', pixel_size, ...
'resolution', [width height], 'centre', [width/2 height/2], 'name', 'mycamera');

cam.T = transl(0, 0, 3) * trotx(pi);

P = mkcube(0.5, 'pose', transl([0, 0, 0]) );    % make a cube of points
P = P(:,1:4);   % just a square

pixels = cam.project(P);
x_pix = pixels(1,:)' - width/2;
y_pix = pixels(2,:)' - height/2;

figure(1), clf
subplot(1,2,1)
cam.plot_camera()
plot_sphere(P, 0.03, 'r');  % display the cube
view(45,40)

subplot(1,2,2)
h1 = plot(x_pix, y_pix, 'r*');
hold on
h2 = plot(x_pix, y_pix, 'b*');
xlabel('u')
ylabel('v')
title('Image Plane (pixels)')
legend('camera frame', 'virtual level frame')
axis ij
axis equal
axis([-width/2, width/2, -height/2, height/2])

% pause;

% 10 seconds
t = 0:0.1:10;

for i=1:length(t)
    % swing the camera around
    cam.T = transl(0, 0, 3) * trotx(pi + pi/16 * sin(t(i))) * troty(pi/16 * cos(t(i)));
    
    T_cam = cam.T.T;
    R_cam_vlc = T_cam(1:3,1:3);
    
    % get the projection
    pix = cam.project(P);
    
    % make pixel locations wrt to image center
    pix(1,:) = pix(1,:) - width/2;
    pix(2,:) = pix(2,:) - height/2;
    
    pix = pix';
    
    uvf = vertcat(pix', [fl_pix fl_pix fl_pix fl_pix]);
    
    % compose eq(14)
    hom = R_cam_vlc * uvf;
    
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
    
    cam.plot_camera()
    pause(0.1)
end

% pause;





