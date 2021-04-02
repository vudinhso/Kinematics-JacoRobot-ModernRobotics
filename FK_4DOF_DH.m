clc; clear all; close all; 
format compact; format shortG

#Manipulator parameters
D1 = .2755;
D2 = .41;
D3 = .2073;
D4 = .16;

#Manipulator home position
M = [   0,1,0,0; ...
        1,0,0,-D2;...
        0,0,-1,D1-D3-D4;...
        0,0,0,1];

#DH parameters
a =     [0,0,D2,0,0]';
alpha = [0,-pi/2,0,-pi/2,0]';
d =     [D1,0,0,0,D3+D4]';
th =    [-pi/2,0,0,0,pi]'; #th init
th =    th + [0,pi/2,0,0,0]';

tic()
T = fcn_Tmatrix(a, alpha, d, th);
T_tot = T(:,:,1);
for i=2:length(a)
    T_tot=T_tot*T(:,:,i);
end
T_tot
delta_time = toc()
