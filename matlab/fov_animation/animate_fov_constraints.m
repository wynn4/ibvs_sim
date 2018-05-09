clc
clear
close all

% load params
param

pn = 0;
pe = 0;
pd = -10;
phi = deg2rad(0);
theta = deg2rad(0);
psi = deg2rad(0);

target_x = 0;
target_y = 0;

target_x_max = 4;
target_y_max = 4;

psi_max = deg2rad(180.0);

% virtual FOV scale factor
v_fov_sf = 0.8;



for i = 1:100
    if i == 1
        t = 0;
        u = [pn, pe, pd, phi, theta, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(1.0);
    else
        t = 1;
        % pick random position for the target within the max target pos
        % interval
        target_x = -target_x_max + (target_x_max - -target_x_max) .* rand(1,1);
        target_y = -target_y_max + (target_y_max - -target_y_max) .* rand(1,1);
        
        % pick random heading angle
        psi = -psi_max + (psi_max - -psi_max) .* rand(1,1);
        
        u = [pn, pe, pd, phi, theta, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(1.5);
        disp("New Target Position")
        
        % get a unit vector in the inertial frame from the target to the
        % vehicle
        vec_target2vehicle = [pn, pe, pd]' - [target_x, target_y, 0]';
        
        % normalize it
        vec_target2vehicle = vec_target2vehicle / norm(vec_target2vehicle);
        
        % rotate into the v1 frame
        R_v_v1 = [cos(psi), sin(psi), 0; -sin(psi), cos(psi), 0; 0, 0 , 1];
        vec_v1 = R_v_v1 * vec_target2vehicle;
        
        % find the virtual roll and pitch angles that would point the camera
        % direclty at the target
        phi_v = asin(vec_v1(2));
        theta_v = atan2(-vec_v1(1), -vec_v1(3));
        
        % calculate max and min allowabe roll and pitch angles to keep the
        % target w/in the FOV
        phi_min = phi_v - ((P.fov_w/2) * v_fov_sf);
        phi_max = phi_v + ((P.fov_w/2) * v_fov_sf);
        
        theta_min = theta_v - ((P.fov_h/2) * v_fov_sf);
        theta_max = theta_v + ((P.fov_h/2) * v_fov_sf);
        
        % draw the aircraft rolled to phi_min
        u = [pn, pe, pd, phi_min, 0, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(0.5);
        
        % draw the aircraft rolled to phi_max
        u = [pn, pe, pd, phi_max, 0, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(0.5);
        
        % draw the aircraft pitched to theta_min
        u = [pn, pe, pd, 0, theta_min, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(0.5);
        
        % draw the aircraft pitched to theta_max
        u = [pn, pe, pd, 0, theta_max, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(0.5);
        
        % draw the aircraft rolled and pitched to min
        u = [pn, pe, pd, phi_min, theta_min, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(0.5);
        
        % draw the aircraft rolled and pitched to max
        u = [pn, pe, pd, phi_max, theta_max, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(0.5);
        
        % draw the aircraft rolled max pitched min
        u = [pn, pe, pd, phi_max, theta_min, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(0.5);
        
        % draw the aircraft rolled min pitched max
        u = [pn, pe, pd, phi_min, theta_max, psi,target_x,target_y,t];
        drawAircraft2(u, V, F, colors, P);
        pause(0.5);
    end
    
    
end

