clc
clear
close all

% load params
param

pn = 0;
pe = 0;
pd = -10;
u = 0;
v = 0;
w = 0;
phi = deg2rad(0);
theta = deg2rad(0);
psi = deg2rad(0);
p = 0;
q = 0;
r = 0;

xc = 0;
yc = 0;
zc = 0;
yawc = 0;
t = 0;
az = 0;
el = -pi/2;

target_x = 5;
target_y = 0;
target_z = 0;

pn_max = 5;
pe_max = 6;
pd = -10;

for i = 1:100
    if i == 1
        t = 0;
    else
        t = 1;
    end
    
    % pick random position for the copter
    r = pn_max + (b-a).*rand(100,1);
end

u = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r, xc, yc, zc, yawc, t, az, el, target_x, target_y, target_z]';

drawAircraft(u, V, F, colors, P)