function [H_dual, H_traslation, H_rotation] = FK_3DOF(theta_1,theta_2, theta_3, l_1, l_2, l_3)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
t1 = [0;0;0;0];
angle_1 = theta_1;

%% The rotation is similiar a rotation vector formation
r1 = rotation_quaternion(angle_1, [0;0;1]);
%% Traslation and then rotation
h1_dual = pose_dual(t1,r1);

%% Second Traslation and rotation

t2 = [0;l_1;0;0];
angle_2 = theta_2;

%% The rotation is similiar a rotation vector formation
r2 = rotation_quaternion(angle_2, [0;0;1]);
%% Traslation and then rotation
h2_dual = pose_dual(t2,r2);

%% third transformation
t3 = [0;l_2;0;0];
angle_3 = theta_3;

%% The rotation is similiar a rotation vector formation
r3 = rotation_quaternion(angle_3, [0;0;1]);

%% Traslation and then rotation
h3_dual = pose_dual(t3,r3);

%% third transformation
t4 = [0;l_3;0;0];
angle_4 = 0;

%% The rotation is similiar a rotation vector formation
r4 = rotation_quaternion(angle_4, [0;0;1]);

%% Traslation and then rotation
h4_dual = pose_dual(t4,r4);

%% get initial Condition based on dual quaternions
H_dual_aux1 = mult_dual(h1_dual, h2_dual);
H_dual_aux2 = mult_dual(H_dual_aux1, h3_dual);
H_dual = mult_dual(H_dual_aux2, h4_dual);

H_traslation =  get_traslatation_dual(H_dual);
H_rotation =  get_rotation_dual(H_dual);

end

