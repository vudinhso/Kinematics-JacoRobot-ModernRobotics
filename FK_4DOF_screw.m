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

#Manipulator screw axis
screw = [   0,0,1,0,0,0;...
            1,0,0,0,D1,0;...
            1,0,0,0,D1,D2;...
            0,0,1,-D2,0,0]';

#Manipulator joint axis
th = [0,pi/2,0,0];

#FK
tic()
T = eye(4,4);
for i=1:length(th)
    T = T*MatrixExp6(VecTose3(screw(:,i)*th(i)));
end
T = T*M

test1 = M;
for i=length(th):-1:1
    test1=fcn_MatrixExp6(screw(:,i), th(i) )*test1;
end
test1

delta_time = toc()
