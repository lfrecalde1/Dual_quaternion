function [H_traslation_init,H_rotation_init] = forward_kinematics_2dof(theta1, theta2, l_1, l_2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% values in order to get the Forward kinematics
t1 = [0;0;0;0];
angle_1 = theta1;

%% Traslation and then rotation
r1 = rotation_quaternion(angle_1, [0;0;1]);
h1_dual = pose_dual(t1,r1);


%% Second transformation
t2 = [0;l_1;0;0];
angle_2 = theta2;

%% The rotation is similiar a rotation vector formation
r2 = rotation_quaternion(angle_2, [0;0;1]);

%% Traslation and then rotation
h2_dual = pose_dual(t2,r2);

%% Second transformation
t3 = [0;l_2;0;0];
angle_3 = 0;

%% The rotation is similiar a rotation vector formation
r3 = rotation_quaternion(angle_3, [0;0;1]);

%% Traslation and then rotation
h3_dual = pose_dual(t3,r3);

%% get initial Condition based on dual quaternions
H_dual_aux = mult_dual(h1_dual, h2_dual);
H_dual = mult_dual(H_dual_aux, h3_dual);

H_traslation =  get_traslatation_dual(H_dual);
H_traslation_init(:, 1) = (H_traslation);
H_rotation_init(:, 1) =  get_rotation_dual(H_dual);
end

