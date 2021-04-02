clc; clear all; close all; 
format compact; format shortG

#Manipulator parameters
D1 = .2755;
D2 = .41;
D3 = .2073;
D4 = .0741;
D5 = .0741;
D6 = .16;
c30 = cosd(30);
s30 = sind(30);
d  = D4/2/c30;

#Manipulator home position
M = [   -s30, 0,c30,D4*s30+D5+D6*c30; ...
         0,-1,0,    -D2;...
         c30,0,s30, D1-D3-D4*c30+D6*s30;...
         0,0,0,  1];

#DH parameters
b = pi/2;
a =     [0,0,D2,0,0,0,0]';
alpha = [0,-b,0,b,-2*pi/3,pi/3,0]';
d =     [D1,0,0,-(D3+d),2*d,d+D6,0]';
th =    [-b,0,0, 0,0,0,-b]';
th = th + [0,0,-pi/2,0,0,0,0]';

tic()
T = fcn_Tmatrix(a, alpha, d, th); 
T_tot = T(:,:,1);
for i=2:length(a)
    T_tot=T_tot*T(:,:,i);
end
T_tot
Tf= [   -s30, 0, c30,   D4*s30+D5+D6*c30;
        c30, 0, s30,    -D2-D3-D4*c30+D6*s30; 
        0, 1, 0,        D1; 
        0,0,0,1]
delta_time = toc()
