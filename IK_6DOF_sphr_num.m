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

#Manipulator screw axis
screw = [   0,0,1,0,0,0;...
            1,0,0,0,D1,0;...
            1,0,0,0,D1,D2;...
            0,0,1,-D2,0,0;...
            0,1,0,D3+D4-D1,0,0;...
            1,0,0,0,-(D3+D4-D1),D2]';

#Manipulator desired configuration
Td= [0, -1, 0, 0;
    0, 0, -1,-0.6441; 
    1, 0, 0, -.0059; 
    0, 0, 0, 1];
    
#Manipulator initial joint axis and configuration
#th = [0,pi/2,0,0];
th = [0,0,0,0];
th = rand(1,6);


#IK
i=0;
err_w = 10;

while i<30 && err_w>1e-6
    T       = FKinSpace(M,screw, th);
    ss_Vb   = MatrixLog6(TransInv(T)*Td);
    Vb      = se3ToVec(ss_Vb);
    Vs      = Adjoint(T)*Vb;
    err_w   = norm(Vs(1:3))
    J       = JacobianSpace(screw, th);
    th      = th + J\Vs;
    i=i+1
end
Td
T
norm(Td-T)
