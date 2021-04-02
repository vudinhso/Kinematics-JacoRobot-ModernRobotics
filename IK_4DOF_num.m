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

#Manipulator desired configuration
Td = ...
[   0, 1, 0, 0;
    0, 0, 1, 0.3673;
    1, 0, 0, -0.1345;
    0, 0, 0, 1]

#Manipulator initial joint axis and configuration
th = rand(1,4);


#IK
i=0;
#{
for i = 1:(size(screw,2))
    V(:,i)      = screw(:,i)*th(i);         #Twist
    ss_V(:,:,i) = VecTose3(V(:,i));         #skew-sym of Twist
    T(:,:,i)    = MatrixExp6(ss_V(:,:,i));  #From skew-sym, which contains screw axis and angle, the transformation matrix can be obtained.
    AdT(:,:,i)  = Adjoint(T(:,:,i));        #From transformation matrix, the adjoint matrix can be obtained to convert twist from space to space
end
AdT_J(:,:,1) = AdT(:,:,1);
for i = 2:(size(screw,2)-1)
    AdT_J(:,:,i) = AdT_J(:,:,i-1)*AdT(:,:,i);
end
   
J = [   screw(:,1), ...
        AdT_J(:,:,1)*screw(:,2), ...
        AdT_J(:,:,2)*screw(:,3), ...
        AdT_J(:,:,3)*screw(:,4)]
#}
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
T
