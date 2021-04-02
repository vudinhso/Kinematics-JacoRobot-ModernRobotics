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

#Manipulator screw axis
w5 = [s30,0,-c30];
w6 = [c30,0, s30];
q5 = [0, -D2, D1-D3-d];
q6 = M(1:3,4)';
screw = [   0,0,1,      0,0,0;...
            1,0,0,      0,D1,0;...
            1,0,0,      0,D1,D2;...
            0,0,1,      -D2,0,0;...
            w5,         cross(w5, -q5);...
            w6,         cross(w6, -q6)]';

#Manipulator joint axis
th = [0,0,-pi/2,0,0,0];

#FK
tic()
T = eye(4,4);
for i=1:length(th)
    T = T*MatrixExp6(VecTose3(screw(:,i)*th(i)));
end

T = T*M

Tf= [   -s30, 0, c30,   D4*s30+D5+D6*c30;
        c30, 0, s30,    -D2-D3-D4*c30+D6*s30; 
        0, 1, 0,        D1; 
        0,0,0,1]

delta_time = toc()
