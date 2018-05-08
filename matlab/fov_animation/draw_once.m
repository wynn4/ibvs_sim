pn = -5;
pe = -5;
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

target_x = 3;%5.7735;
target_y = 3;%7.002;
target_z = 0;

u = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r, xc, yc, zc, yawc, t, az, el, target_x, target_y, target_z]';

drawAircraft(u, V, F, colors, P)