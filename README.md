# Kinematics-JacoRobot-ModernRobotics
Use of screw theory instead of modified DH
For forward kinematics with modified DH-parameters: 
- FK_4DOF_DH.m: calculate the FK for the Jaco Robot with 4-DOFs, 
                with the modified DH-parameters.
- FK_6DOF_curv_DH.m: 
                calculate the FK for the Jaco Robot with 6-DOFs, 
                with the curved wrist, 
                with the modified DH-parameters.
- FK_6DOF_sphr_DH.m: 
                calculate the FK for the Jaco Robot with 6-DOFs, 
                with the spherical wrist, 
                with the modified DH-parameters.
- fcn_Tmatrix.m: Build the transformation matrix SE(3) based on the DH-parameters

For forward kinematics with screw theory: 
- FK_4DOF_screw.m
- FK_6DOF_curv_screw.m
- FK_6DOF_sphr_screw.m

Screw theory tends to use a lot of functions, which are the following:
- VecTose3.m: Convert a 6d twist vector to a skew-symmetric matrix in se(3)
  - VecToso3.m: Convert a 3d vector to a skew-symmetric matrix in so(3)
- MatrixExp6.m: Calculate the configuration T from a matrix se(3), which is calculate from a 6D twist. 
  - so3ToVec.m: Convert a skew-symmetric matrix so(3) to a 3d vector
  - NearZero.m: Check if the norm of a vector is close to zero
  - AxisAng3.m: Calculate. the normalized vector and the norm of the vector
  - MatrixExp3.m: Calculate the rotation matrix from a skew-symmetric matrix

A more compact library is as follows: 
- fcn_MatrixExp6.m: takes as argument the screw axis S and the angle theta
  - VecToso3.m: takes a 3d vector as argument, return the so(3) skew-symmetric matrix.

For inverse kinematics with screw theory: 
- IK_4DOF_screw.m
- IK_6DOF_curv_screw.m
- IK_6DOF_sphr_screw.m

It uses the following function:
- FKinSpace: calculate the forward kinematics
  - MatrixExp6.m
  - VecTose3.m
- MatrixLog6.m: from a transformation matrix, return a skew-symmetric matrix se3
  - TransInv.m: calculate the inverse of a transformation matrix
- se3ToVec.m: convert a skew-symmetric matrix se3 to a twist
- Adjoint.m: Calculate the adjoint matrix from a transformation matrix, to convert twist from space to space. 
  - TransToRp.m
  - VecToso3.m
- JacobianSpace.m: Calculate the space Jacobian from a matrix containing the screw and a vector containing the angle. 
  - MatrixExp6.m
  - VecTose3.m
  - Adjoint.m
