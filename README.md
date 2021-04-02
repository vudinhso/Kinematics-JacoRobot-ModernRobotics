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
