function drawAircraft(uu,V,F,patchcolors,P)

persistent dA  % dA => struct that will help us distinguish data for each distinct agent
               % dA(1) => data for agent 1, etc...  

    NN = 0;
    for i = 1:P.num_agents,
        % process inputs to function
        dA(i).pn       = uu(NN+1);  % inertial North position     
        dA(i).pe       = uu(NN+2);  % inertial East position
        dA(i).pd       = uu(NN+3);           
        dA(i).u        = uu(NN+4);       
        dA(i).v        = uu(NN+5);       
        dA(i).w        = uu(NN+6);       
        dA(i).phi      = uu(NN+7);  % roll angle         
        dA(i).theta    = uu(NN+8);  % pitch angle     
        dA(i).psi      = uu(NN+9);  % yaw angle     
        dA(i).p        = uu(NN+10); % roll rate
        dA(i).q        = uu(NN+11); % pitch rate     
        dA(i).r        = uu(NN+12); % yaw rate  
        
        NN = NN + 12;
    end
    for i = 1:P.num_agents,
        dA(i).x_c      = uu(NN+1); % x command 
        dA(i).y_c      = uu(NN+2); % y command
        dA(i).z_c      = uu(NN+3); % z command
        dA(i).yaw_c    = uu(NN+4); % yaw command
        
        NN = NN + 4;
    end
    t        = uu(NN+1); % time
    NN = NN + 1;
    for i = 1:P.num_agents,
        dA(i).az       = uu(NN+1); % azmuth for camera
        dA(i).el       = uu(NN+2); % elevator for camera
        
        NN = NN + 2;
    end
    target   = [uu(NN+1); uu(NN+2); uu(NN+3)];  %target x, y, z position


    % define persistent variables 
    persistent spacecraft_handle;
    persistent commanded_position_handle;
    persistent target_handle;    % handle for target
    persistent fov_handle;       % handle for camera field-of-view
    view_range = 5;
    close_enough_tolerance = 0.9;
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        axis square
        axis equal
        axis([-20,20,-20,20,0,15]);
        
        hold on;
        for i = 1:P.num_agents
            spacecraft_handle(i) = drawSpacecraftBody(V,F,patchcolors,...
                                                   dA(i).pn,dA(i).pe,dA(i).pd,dA(i).phi,dA(i).theta,dA(i).psi,...
                                                   [],'normal');
            commanded_position_handle(i) = drawCommandedPosition(dA(i).x_c,dA(i).y_c,dA(i).z_c,dA(i).yaw_c,...
                                                   []);
            fov_handle(i) = drawFov(dA(i).pn, dA(i).pe, dA(i).pd, dA(i).phi, dA(i).theta, dA(i).psi,dA(i).az,dA(i).el,P.cam_fov,[],'normal');
        end
        target_handle = drawTarget(target, P.target_size, [], 'normal');
        title('Spacecraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        grid on;
        view(32,47)  % set the view angle for figure
        for i = 1:P.num_agents,
            dA(i).center = [dA(i).pe;dA(i).pn;dA(i).pd];
        end
        hold on
        
    % at every other time step, redraw base and rod
    else
        for i = 1:P.num_agents,
            dA(i).pos = [dA(i).pe;dA(i).pn;dA(i).pd];
            close_enough_tolerance*[view_range;view_range;view_range] - abs(dA(i).pos - dA(i).center);
            if (min(close_enough_tolerance*[view_range;view_range;view_range] - abs(dA(i).pos-dA(i).center)) < 0)
                dA(i).center = dA(i).pos;
            end
            drawSpacecraftBody(V,F,patchcolors,...
                               dA(i).pn,dA(i).pe,dA(i).pd,dA(i).phi,dA(i).theta,dA(i).psi,...
                               spacecraft_handle(i));
            hold on
            drawCommandedPosition(dA(i).x_c,dA(i).y_c,dA(i).z_c,dA(i).yaw_c,...
                               commanded_position_handle(i));
            hold on
            drawFov(dA(i).pn, dA(i).pe, dA(i).pd, dA(i).phi, dA(i).theta, dA(i).psi,dA(i).az,dA(i).el,P.cam_fov,fov_handle(i));
            hold on
        end
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
function handle = drawFov(pn, pe, pd, phi, theta, psi,az,el,cam_fov,handle, mode)
                           

    %-------vertices and faces for camera field-of-view --------------
    % vertices
    % define unit vectors along fov in the camera gimbal frame
    pts = [...
        cos(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2);...
        cos(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2);...
        cos(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2);...
        cos(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2);...
        ]';
    % transform from gimbal coordinates to the vehicle coordinates
    pts = Rot_v_to_b(phi,theta,psi)'*Rot_b_to_g(az,el)'*pts;

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
                [pn-pd*pts(1,i)/pts(3,i), pe-pd*pts(2,i)/pts(3,i), 0];...
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
  