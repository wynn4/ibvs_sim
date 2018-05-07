clear;

% add all subdirectories to current path
addpath(genpath(pwd));

P.Ts = 0.01;

P.Tout = P.Ts;

% physical parameters of airframe
P.gravity = 9.81;
P.mass    = 3.81;
P.Jxx     = 0.060224;
P.Jyy     = 0.122198;
P.Jzz     = 0.132166;

% % Params from paper
% P.mass    = 0.468;
% P.Jxx     = 4.856e-3;
% P.Jyy     = 4.856e-3;
% P.Jzz     = 8.801e-3;

P.L  = 0.25;
P.k1 = 1; %2.98*10e-6;
P.k2 = 1; %2.98*10e-6;
P.mu = 1;

% sketch parameters
P.nRotors = 4;
P.num_agents = 1;
for i=1:P.num_agents,
    sketch_copter;
end

% first cut at initial conditions
for i = 1:P.num_agents,
    P.pn0(i)    = 0;  % initial North position
    P.pe0(i)    = 0; %20*(-1)^(i+1);  % initial East position
    P.pd0(i)    = -30;  % initial Down position (negative altitude) 90m~300ft
    P.u0(i)     = 0;  % initial velocity along body x-axis
    P.v0(i)     = 0;  % initial velocity along body y-axis
    P.w0(i)     = 0;  % initial velocity along body z-axis
    P.phi0(i)   = 0;  % initial roll angle
    P.theta0(i) = 0;  % initial pitch angle
    P.psi0(i)   = 0;  % initial yaw angle
    P.p0(i)     = 0;  % initial body frame roll rate
    P.q0(i)     = 0;  % initial body frame pitch rate
    P.r0(i)     = 0;  % initial body frame yaw rate
end

% compute gains
compute_gains;

% time constant for dirty derivative filter
P.tau = 0.15;


% target parameters
P.target_velocity = 5;  % (m/s)
P.target_size = 0.5;          % size of target 

% adaptive parameter
P.gamma_theta = 3000;
P.alpha = 10;

% gimbal parameters
P.az0 = 0;      % initial azimuth angle
P.el0 = -pi/2;  % initial elevation angle (pointing down)
P.az_limit = 720*(pi/180);  % azimuth angle limit
P.el_limit = 180*(pi/180);  % elevation angle limit
P.azdot_limit = 2;
P.eldot_limit = 2;

P.az_gain  = 1;  % gain on azimuth dynamics (azdot = az_gain*u_az)
P.el_gain  = 1;  % gain on elevation dynamics (eldot = el_gain*u_el)
P.k_az     = 10; % proportional control gain for gimbal azimuth
P.k_el     = 10; % proportional control gain for gimbal elevation

% camera parameters
P.cam_fps = 30;  % frames per second 
P.cam_pix = 800;                      % size of (square) pixel array
P.fov_w   = 70*(pi/180);              % field of view of camera
P.fov_h   = 40*(pi/180);
P.f = (P.cam_pix/2)/tan(P.fov_w/2); % focal range in pixel
P.pixelnoise = 0;                     % (pixels) - variance of the pixel noise
P.pixel_to_meter = 0.000003226;       % 1 pixel = 0.000003226 m = 0.003226 mm

% measurement model for GPS (used in geolocation algorithm)
P.sigma_measurement_n = 0.547; % standard deviation of gps measurement error in m
P.sigma_measurement_e = 0.547; % standard deviation of gps measurement error in m
P.sigma_measurement_h = 1.14; % standard deviation of gps measurement error in m
