clc
clear
close all

phi = 0.0;
theta = deg2rad(-45.0);
psi = 0.0;

R_i_b = rotx(phi)' * roty(theta)' * rotz(psi)';

R_b_c = [0 1 0; -1 0 0; 0 0 1];

d_i_b = [0, 0, 0]';

d_b_c = [0 sqrt(2) 0]';

T_i_b = [R_i_b, d_i_b;
         0, 0, 0, 1];
     
T_b_c = [R_b_c, d_b_c;
         0, 0, 0, 1];
     
T_i_c = T_b_c * T_i_b;
     

     
ell_c = [0, 0, 1, 1]';

k_i = [0, 0, 1]';

h = 3.0;

T_c_i = [T_i_c(1:3,1:3)', -T_i_c(1:3,1:3)' * T_i_c(1:3,4); 0, 0, 0, 1];

term = T_c_i * ell_c;

term = term(1:3);
num = term;
den = dot(k_i, term);

Pmav = [0 0 0]' - h * (num/den)

R_i_c = R_b_c * R_i_b;
R_c_i = R_i_c';

num = R_c_i * ell_c(1:3);
den = dot(k_i, num);

pmav = [0 0 0]' - h * (num/den)