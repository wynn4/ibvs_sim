function drawPlanarQuad(u)

    % process inputs to function
    theta = u(1);
    Z = u(2);
    h = u(3);
    Z_target = u(4);
    t = u(5);
    
    L = 2;
    %w = 1;
    FOV = deg2rad(45.0);
    
    
    % define persistent variables 
    persistent body_handle
    persistent target_handle
    persistent line_handle
    persistent fov_handle
    persistent los_handle
    persistent accel_handle
       
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        plot([-6,6],[0,0],'k'); % plot track
        axis equal
        axis([-6 6 -1 20])
        hold on
        
        target_handle = drawTarget_(Z_target, []);
        line_handle = drawLine_(theta, Z, h, L, []);
        fov_handle = drawFOV_(theta, Z, h, FOV, []);
        los_handle = draw_Target_LOS_(theta, Z, h, Z_target, []);
        accel_handle = drawACCEL_region_(theta, Z, h, FOV, Z_target, []);
        body_handle = drawBody_(theta, Z, h, L, []);
        
        
        % axis([-6*L, 6*L, -1*L, 11*L]);
        xlabel('Z_v Position')
        ylabel('h Position')
        
    % at every other time step, redraw planar VTOL
    else 
        drawTarget_(Z_target, target_handle);
        drawLine_(theta, Z, h, L, line_handle);
        drawFOV_(theta, Z, h, FOV, fov_handle);
        draw_Target_LOS_(theta, Z, h, Z_target, los_handle)
        drawACCEL_region_(theta, Z, h, FOV, Z_target, accel_handle)
        drawBody_(theta, Z, h, L, body_handle);
        
        
    end
end

   
%
%=======================================================================
% drawBody_
% draw the planar VTOL body
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBody_(theta, Z, h, L, handle)
  
%   pts = [...
%       L/2, L/2;...
%       L/2, L/16;...
%       3*L/2, L/16;...
%       3*L/2, L/8;...
%       5*L/2, L/8;...
%       5*L/2, -L/8;...
%       3*L/2, -L/8;...
%       3*L/2, -L/16;...
%       L/2, -L/16;...
%       L/2, -L/2;...
%       -L/2, -L/2;...
%       -L/2, -L/16;...
%       -3*L/2, -L/16;...
%       -3*L/2, -L/8;...
%       -5*L/2, -L/8;...
%       -5*L/2, L/8;...
%       -3*L/2, L/8;...
%       -3*L/2, L/16;...
%       -L/2, L/16;...
%       -L/2, L/2;...
%       ]';
  pts = [...
         -0.01*L, 0;
         -0.05*L, -0.05*L;
         0.05*L, -0.05*L;
         0.01*L, 0;
         0.5*L, 0;
         0.5*L, 0.1*L;
         0.4*L, 0.1*L;
         0.4*L, 0.05*L;
         0, 0.05*L;
         0, 0.5*L;
         0, 0.05*L;
         -0.4*L, 0.05*L;
         -0.4*L, 0.1*L;
         -0.5*L, 0.1*L;
         -0.5*L, 0;...
         %-0.01, 0;...
         ]';
     
  pts = [cos(theta), -sin(theta); sin(theta), cos(theta)]*pts;
  X = pts(1,:) + Z;
  Y = pts(2,:) + h;
  
  if isempty(handle)
    handle = fill(X,Y,'k');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

%=======================================================================
% drawTarget_
% draw the Target
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawTarget_(Z_target, handle)
 
%   pts = [...
%       L/8, L/4;...
%       L/8, 0;...
%       -L/8, 0;...
%       -L/8, L/4;...
%       ]';

  pts = [...
         0, 0;
         0.1, 0;
         0.1, 0.2;
         -0.1, 0.2;
         -0.1, 0;
         0, 0;
         ...
         ]';
    
  X = pts(1,:);
  Y = pts(2,:);
  X = X + Z_target;
  
  if isempty(handle)
    handle = fill(X,Y,'r');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

%=======================================================================
% drawTarget_Line
% draw the line pointing toward the Target
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawLine_(theta, Z, h, L, handle)
 
  pts = [...
      Z, h;...
      0, 0;...
      ]';
  
  % pts(1,1) = pts(1,1) + Z;
  % pts(2,1) = pts(2,1) + h;
  pts(1,2) = h*tan(theta) + Z;
    
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = fill(X,Y,'b--');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

function handle = drawFOV_(theta, Z, h, FOV, handle)
 
  pts = [...
      0, 0;...
      Z, h;...
      0, 0;...
      ]';
  
  % pts(1,1) = pts(1,1) + Z;
  % pts(2,1) = pts(2,1) + h;
  pts(1,1) = h*tan(theta - FOV/2) + Z;
  pts(1,3) = h*tan(theta + FOV/2) + Z;
    
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = plot(X,Y,'b-');
    % hold on
    % alpha(0.1)
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

function handle = draw_Target_LOS_(theta, Z, h, Z_target, handle)
 
  pts = [...
      Z, h;...
      Z_target, 0;...
      ]';
  
  % pts(1,1) = pts(1,1) + Z;
  % pts(2,1) = pts(2,1) + h;
  % pts(1,2) = h*tan(theta) + Z;
    
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = plot(X,Y,'r--');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

function handle = drawACCEL_region_(theta, Z, h, FOV, Z_target, handle)
 
  pts = [...
      0, 20;...
      Z, h;...
      0, 20;...
      ]';
  % get the FOV vector to the target
  vec = [Z_target - Z, h]';
  angle = atan(vec(1)/vec(2));
  % pts(1,1) = pts(1,1) + Z;
  % pts(2,1) = pts(2,1) + h;
  pts(1,1) = (20-h)*tan(-angle - FOV/2) + Z;
  pts(1,3) = (20-h)*tan(-angle + FOV/2) + Z;
    
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = plot(X,Y,'g-');
    % hold on
    % alpha(0.1)
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end