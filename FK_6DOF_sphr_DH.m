clc; clear all; close all; 
format compact; format shortG

#Manipulator parameters
D1 = .2755;
D2 = .41;
D3 = .2073;
D4 = .0741;
D5 = .0741;
D6 = .16;

#Manipulator home position
M = [   0,0,1,  D5+D6; ...
        0,-1,0, -D2;...
        1,0,0,  D1-D3-D4;...
        0,0,0,  1];

#DH parameters
b = pi/2;
a =     [0,0,D2,0,0,0,0]';
alpha = [0,-b,0,b,-b,-b,0]';
d =     [D1,0,0,-(D3+D4),0,0,D5+D6]';
th =    [-b,0,0, b,-b,b,-b]';
th = th + [0,0,0,-pi/2,0,0,0]';

tic()
T = fcn_Tmatrix(a, alpha, d, th); 
T_tot = T(:,:,1);
for i=2:length(a)
    T_tot=T_tot*T(:,:,i);
end
T_tot
delta_time = toc()
