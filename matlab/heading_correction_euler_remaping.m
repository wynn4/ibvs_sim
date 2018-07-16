clc
clear
close all

% Input Euler Angles
phi = 15;
theta = -10;
psi = 45;

% convert degrees to radians
phi = deg2rad(phi);
theta = deg2rad(theta);
psi = deg2rad(psi);

% pre-evaluate sines and cosines
sphi = sin(phi);
cphi = cos(phi);

stheta = sin(theta);
ctheta = cos(theta);

spsi = sin(psi);
cpsi = cos(psi);


RtimesZ = [cphi*stheta*cpsi + sphi*spsi;
           cphi*stheta*spsi - sphi*cpsi;
           cphi*ctheta];

z_b_i = RtimesZ;


% Now we desire to be yawed to 10 degrees instead of 45 but we still want
% our z-axis to be the same in the inertial frame


x = fsolve(@mfun, [0, 0]);

z_new = [cos(x(1))*sin(x(2))*cos(deg2rad(10.0)) + sin(x(1))*sin(deg2rad(10.0));
         cos(x(1))*sin(x(2))*sin(deg2rad(10.0)) - sin(x(1))*cos(deg2rad(10.0));
         cos(x(1))*cos(x(2))]
     
z_original = z_b_i

rad2deg(x(1))
rad2deg(x(2))



function F = mfun(euler_angles)

phi = euler_angles(1);
theta = euler_angles(2);
psi = deg2rad(10.0);

% pre-evaluate sines and cosines
sphi = sin(phi);
cphi = cos(phi);

stheta = sin(theta);
ctheta = cos(theta);

spsi = sin(psi);
cpsi = cos(psi);


z_b_i = [0.064408790885269, -0.301616612899170, 0.951251242564198]';

vec = [cphi*stheta*cpsi + sphi*spsi;
           cphi*stheta*spsi - sphi*cpsi;
           cphi*ctheta];
F = vec - z_b_i;
end