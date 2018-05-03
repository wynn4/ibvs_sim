
%Draw Hex
% parameters for drawing aircraft
% scale size
scale = 5;
    
spoke.h = 0.02;
spoke.w = 0.02;
spoke.l = 0.25;

center.above = 0.1;
center.below = 0.1;
center.r = 0.1;

angle = 2*pi/P.nRotors;

% Define the vertices (physical location of vertices
  V_rotor = [...
	 0,       -spoke.w/2,  spoke.h/2; % inside 4 of spoke
     0,        spoke.w/2,  spoke.h/2; % inside 4 of spoke
     0,        spoke.w/2, -spoke.h/2; % inside 4 of spoke
     0,       -spoke.w/2, -spoke.h/2; % inside 4 of spoke
     spoke.l, -spoke.w/2,  spoke.h/2; % outside 4 of spoke
     spoke.l,  spoke.w/2,  spoke.h/2; % outside 4 of spoke
     spoke.l,  spoke.w/2, -spoke.h/2; % outside 4 of spoke
     spoke.l, -spoke.w/2, -spoke.h/2; % outside 4 of spoke
     0,        0,          -center.above; % tip
     cos(angle/2)*center.r,  sin(angle/2)*center.r,  center.below; % base1
     cos(angle/2)*center.r, -sin(angle/2)*center.r,  center.below; % base2
     0, 0,  center.below; % base_center
  ];

% define faces as a list of vertices numbered above
  F_rotor = [...
        1,2,3,4; % strut inside
        5,6,7,8; % strut outside
        1,2,6,5; % strut top
        2,3,7,6; % strut side
        3,4,8,7; % strut bottom
        4,1,5,8; % strut side
        9,10,11,11; % center face 
        12,10,11,11]; % center bottom 

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];
  mywhite = [1, 1, 1];

  colors_rotor = [...
     myred;...      
     myred;
     mygreen;...    
     myblue;...     
     myyellow;...   
     mycyan;... 
     mywhite;... 
     mywhite;... 
    ];

  V_rotor = scale*V_rotor;   % rescale vertices

  V = [];
  F = [];
  colors = [];
  for rotor = 0:P.nRotors-1
    angle = rotor*2*pi/P.nRotors;
    R = [cos(angle), sin(angle),0;
        -sin(angle), cos(angle),0;
         0,0,1];
    V = [V;V_rotor*R];
    F = [F;F_rotor+rotor*size(V_rotor,1)];
    colors = [colors;colors_rotor];
  end
  
  colors(3,:) = myred; 
  colors(4,:) = myred; 
  colors(5,:) = myred; 
  colors(6,:) = myred; 
  