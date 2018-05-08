function drawAircraft2(uu,V,F,patchcolors,P)

NN = 0;
% process inputs to function
pn       = uu(NN+1);  % inertial North position
pe       = uu(NN+2);  % inertial East position
pd       = uu(NN+3);
phi      = uu(NN+4);  % roll angle
theta    = uu(NN+5);  % pitch angle
psi      = uu(NN+6);  % yaw angle

target_x = uu(NN+7);
target_y = uu(NN+8);
target_z = 0;
t = uu(NN+9);

az = 0;
el = -pi/2;

target   = [target_x; target_y; target_z];  %target x, y, z position


% define persistent variables
persistent spacecraft_handle;
persistent target_handle;    % handle for target
persistent fov_handle;       % handle for camera field-of-view
persistent accel_region;
persistent line_handle
persistent k_axis_handle;

view_range = 5;
close_enough_tolerance = 0.9;

% first time function is called, initialize plot and persistent vars
if t==0,
    figure(1), clf
    axis equal
    axis([-10,10,-10,10,0,15]);
    % axis([-7, 7, -7, 7, 0, 15]);
    
    hold on;
    
    spacecraft_handle = drawSpacecraftBody(V,F,patchcolors,...
        pn,pe,pd,phi,theta,psi,...
        [],'normal');
    fov_handle = drawFov(pn, pe, pd, phi, theta, psi,az,el,P.fov_w, P.fov_h, [],'normal');
    accel_region = drawAccelRegion(pn, pe, pd, psi, P.fov_w, P.fov_h, target, [], 'normal');
    vec_targ2vehicle = [pn; pe; pd] - target;
    vec_targ2vehicle = vec_targ2vehicle / norm(vec_targ2vehicle);
    line_handle = drawAccelLine(pn, pe, pd, vec_targ2vehicle, target, []);
    k_axis_handle = draw_K_axis(pn, pe, pd, phi, theta, psi, []);
    
    
    target_handle = drawTarget(target, P.target_size, [], 'normal');
    title('Spacecraft')
    xlabel('East')
    ylabel('North')
    zlabel('-Down')
    grid on;
    view(25,15)  % set the view angle for figure
    hold on
    
    % at every other time step, redraw base and rod
else
    
    % pos = [pe;pn;pd];
    % close_enough_tolerance*[view_range;view_range;view_range] - abs(pos - center);
    % if (min(close_enough_tolerance*[view_range;view_range;view_range] - abs(pos-center)) < 0)
        % center = pos;
    % end
    drawSpacecraftBody(V,F,patchcolors,...
        pn,pe,pd,phi,theta,psi,...
        spacecraft_handle);
    hold on
    drawFov(pn, pe, pd, phi, theta, psi,az,el,P.fov_w, P.fov_h,fov_handle);
    hold on
    
    drawAccelRegion(pn, pe, pd, psi, P.fov_w, P.fov_h, target, accel_region);
    hold on
    vec_targ2vehicle = [pn; pe; pd] - target;
    vec_targ2vehicle = vec_targ2vehicle / norm(vec_targ2vehicle);
    drawAccelLine(pn, pe, pd, vec_targ2vehicle, target, line_handle);
    hold on
    draw_K_axis(pn, pe, pd, phi, theta, psi, k_axis_handle);
    hold on
    
    drawTarget(target, P.target_size, target_handle);
    hold on
end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawFov(pn, pe, pd, phi, theta, psi,az,el,fov_w,fov_h,handle, mode)
                           

    %-------vertices and faces for camera field-of-view --------------
    % vertices
    % define unit vectors along fov in the camera-body frame
    % this is derived from the geometry of a pin-hole camera's fov
    value = sqrt(1 + tan(fov_w/2)^2 + tan(fov_h/2)^2);
    ix = tan(fov_w/2)/value;
    iy = tan(fov_h/2)/value;
    iz = 1/value;
    pts = [ ix,  -iy, iz     % top-right
            -ix, -iy, iz     % top-left
            -ix, iy, iz     % bot-left
            ix,  iy, iz ]'; % bot-right
    
    R_g_c = [0 1 0; 0 0 1; 1 0 0];
    % transform from gimbal coordinates to the vehicle coordinates
    pts = Rot_v_to_b(phi,theta,psi)'*Rot_b_to_g(az,el)'* R_g_c' * pts;

    % first vertex is at center of MAV vehicle frame
    Vert = [pn, pe, pd];  
    % project field of view lines onto ground plane and make correction
    % when the projection is above the horizon
    for i=1:4,
        % alpha is the angle that the field-of-view line makes with horizon
        alpha = atan2(pts(3,i),norm(pts(1:2,i)));
        if alpha > 0,
            % this is the normal case when the field-of-view line
            % intersects ground plane
            Vert = [...
                Vert;...
                [(pn-pd)*(pts(1,i)/pts(3,i)), (pe-pd)*(pts(2,i)/pts(3,i)), 0];...
                ];
        else
            % this is when the field-of-view line is above the horizon.  In
            % this case, extend to a finite, but far away (9999) location.
            Vert = [...
                Vert;...
                [pn+9999*pts(1,i), pe+9999*pts(2,i),0];...
            ];
        end
    end

    Faces = [...
          1, 1, 2, 2;... % x-y face
          1, 1, 3, 3;... % x-y face
          1, 1, 4, 4;... % x-y face
          1, 1, 5, 5;... % x-y face
          2, 3, 4, 5;... % x-y face
        ];

    edgecolor      = [1, 1, 1]; % black
    footprintcolor = [1,1,0];%[1,0,1];%[1,1,0];
    colors = [edgecolor; edgecolor; edgecolor; edgecolor; footprintcolor];  

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  Vert = Vert*R;

  if isempty(handle),
    handle = patch('Vertices', Vert, 'Faces', Faces,...
                 'FaceVertexCData',colors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',Vert,'Faces',Faces);
  end
  
end

function handle = drawAccelRegion(pn, pe, pd, psi, fov_w, fov_h, target, handle, mode)

%-------vertices and faces for camera field-of-view --------------
    % vertices
    % unit vectors same as for fov
    % this is derived from the geometry of a pin-hole camera's fov
    value = sqrt(1 + tan(fov_w/2)^2 + tan(fov_h/2)^2);
    ix = tan(fov_w/2)/value;
    iy = tan(fov_h/2)/value;
    iz = 1/value;
    pts = [ ix,  -iy, iz     % top-right
            -ix, -iy, iz     % top-left
            -ix, iy, iz     % bot-left
            ix,  iy, iz ]'; % bot-right
    
    % but we take the negative to project above the quad
    pts = -pts;
    
    % get a unit vector in the inertial frame from the target to the
    % vehicle
    vec_target2vehicle = [pn, pe, pd]' - target;
    
    % normalize it
    vec_target2vehicle = vec_target2vehicle / norm(vec_target2vehicle);
    
    % rotate into the v1 frame
    R_v_v1 = [cos(psi), sin(psi), 0; -sin(psi), cos(psi), 0; 0, 0 , 1];
    vec_v1 = R_v_v1 * vec_target2vehicle;
    
%     % find the total angle between this line and the vehicle -k axis
%     angle = acos(dot([0; 0; -1], vec_v1));
%     
%     % find axis that is normal to vec_v1
%     %m_axis = [vec_v1(2), -vec_v1(1), 0]';
%     m_axis = cross(vec_v1, [0, 0, -1]');
%     
%     % normalize it
%     m_axis = m_axis / norm(m_axis);
%     
%     u = m_axis(1);
%     v = m_axis(2);
%     w = m_axis(3);
%     
%     % define rotation about m_axis by angle 
%     R = [u^2 + (v^2 + w^2)*cos(angle), u*v*(1-cos(angle))-w*sin(angle), u*w*(1-cos(angle))+v*sin(angle);
%          u*v*(1-cos(angle))+w*sin(angle), v^2 + (u^2 + w^2)*cos(angle), v*w*(1-cos(angle))-u*sin(angle);
%          u*w*(1-cos(angle))-v*sin(angle), v*w*(1-cos(angle))+u*sin(angle), w^2 + (u^2 + v^2)*cos(angle)];
    
    % find the virtual roll and pitch angles that would point the camera
    % direclty at the target
    phi = asin(vec_v1(2));
    theta = atan2(-vec_v1(1), -vec_v1(3));
     
    %R_g_c = [0 1 0; 0 0 1; 1 0 0];
    % transform from gimbal coordinates to the vehicle coordinates
    %pts = Rot_v_to_b(0,0,psi)'*Rot_b_to_g(0,0)'* R_g_c' * pts;
    
    R_c_v1 = [0 -1 0; 1 0 0; 0 0 1];
    pts = Rot_v_to_b(phi,theta,psi)'*R_c_v1 * pts;
    
    
    
    % first vertex is at center of MAV vehicle frame
    Vert = [pn, pe, pd];  
    % project field of view lines onto ground plane and make correction
    % when the projection is above the horizon
    for i=1:4,
        
        Vert = [...
            Vert;...
            [pn+5*pts(1,i), pe+5*pts(2,i),pd+5*pts(3,i)];...
            ];
        
    end
    
    Faces = [...
          1, 1, 2, 2;... % x-y face
          1, 1, 3, 3;... % x-y face
          1, 1, 4, 4;... % x-y face
          1, 1, 5, 5;... % x-y face
          2, 3, 4, 5;... % x-y face
        ];

    edgecolor      = [0, 0, 1]; % black
    footprintcolor = [0,1,0];%[1,0,1];%[1,1,0];
    colors = [edgecolor; edgecolor; edgecolor; edgecolor; footprintcolor];  

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  Vert = Vert*R;

  if isempty(handle),
    handle = patch('Vertices', Vert, 'Faces', Faces,...
                 'FaceVertexCData',colors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',Vert,'Faces',Faces);
  end
    
    
end

function handle = drawAccelLine(pn, pe, pd, vec_targ2vehicle, target_pos, handle)
    % start = [pn, pe, pd]';
    % termination = [pn, pe, pd]' + vec_targ2vehicle*6;
    start = target_pos;
    termination = start + vec_targ2vehicle*20;
    
    R = [...
         0, 1, 0;...
         1, 0, 0;...
         0, 0, -1;...
        ];
    
    start = R * start;
    termination = R * termination;
  
    
    line = [start'; termination'];
    if isempty(handle)
        handle = plot3(line(:,1), line(:,2), line(:,3), '--r');
    else
        set(handle, 'XData', line(:,1), 'YData', line(:,2), 'ZData', line(:,3));
    end
end

function handle = draw_K_axis(pn, pe, pd, phi, theta, psi, handle)

  start = [pn; pe; pd];
  termination = Rot_v_to_b(phi, theta, psi)'*[0 0 -1]';
  termination = start + termination * 20;
  
  R = [...
         0, 1, 0;...
         1, 0, 0;...
         0, 0, -1;...
        ];
    
  start = R * start;
  termination = R * termination;
  
  line = [start'; termination'];
  if isempty(handle)
      handle = plot3(line(:,1), line(:,2), line(:,3), '-k');
  else
      set(handle, 'XData', line(:,1), 'YData', line(:,2), 'ZData', line(:,3));
  end

end

function handle = drawSpacecraftBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = translate(V', pn, pe, pd)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  if isempty(handle),
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end

function handle = drawCommandedPosition(x_c, y_c, z_c, yaw_c,...
                                     handle)
  V = translate([0; 0; 0], x_c, y_c, z_c)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  if isempty(handle),
  handle = plot3(V(1),V(2),V(3),'*','markersize',10);
  else
    set(handle,'xdata',V(1),'ydata',V(2),'zdata',V(3));
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_yaw*R_pitch*R_roll;
  % rotate vertices
  XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

function handle=drawTarget(z, R, handle, mode);
  th = 0:.1:2*pi;
  X = z(1)+ R*cos(th);
  Y = z(2)+ R*sin(th);
  Z = z(3)*ones(length(th));
  
  if isempty(handle),
    handle = fill(Y, X, 'r', 'EraseMode', mode);
  else
    set(handle,'XData',Y,'YData',X);
  end
end

function R = Rot_v_to_b(phi,theta,psi);
% Rotation matrix from body coordinates to vehicle coordinates
Rot_v_to_v1 = [...
    cos(psi), sin(psi), 0;...
    -sin(psi), cos(psi), 0;...
    0, 0, 1;...
    ];
    
Rot_v1_to_v2 = [...
    cos(theta), 0, -sin(theta);...
    0, 1, 0;...
    sin(theta), 0, cos(theta);...
    ];
    
Rot_v2_to_b = [...
    1, 0, 0;...
    0, cos(phi), sin(phi);...
    0, -sin(phi), cos(phi);...
    ];
    
R = Rot_v2_to_b * Rot_v1_to_v2 * Rot_v_to_v1;
end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_b_to_g(az,el);
% Rotation matrix from body coordinates to gimbal coordinates
Rot_b_to_g1 = [...
    cos(az), sin(az), 0;...
    -sin(az), cos(az), 0;...
    0, 0, 1;...
    ];

Rot_g1_to_g = [...
    cos(el), 0, -sin(el);...
    0, 1, 0;...
    sin(el), 0, cos(el);...
    ];

R = Rot_g1_to_g * Rot_b_to_g1;
end
  