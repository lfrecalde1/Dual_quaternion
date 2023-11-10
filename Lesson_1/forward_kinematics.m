function [H_traslation_init,H_rotation_init] = forward_kinematics(theta, l_1)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% values in order to get the Forward kinematics
t1 = [0;0;0;0];
angle_1 = theta;

%% Traslation and then rotation
r1 = rotation_quaternion(angle_1, [0;0;1]);
h1_dual = pose_dual(t1,r1);

t2 = [0;l_1;0;0];
angle_2 = 0;

%% The rotation is similiar a rotation vector formation
r2 = rotation_quaternion(angle_2, [0;0;1]);

%% Traslation and then rotation
h2_dual = pose_dual(t2,r2);

%% get initial Condition based on dual quaternions
H_dual = mult_dual(h1_dual, h2_dual);
H_traslation =  get_traslatation_dual(H_dual);
H_traslation_init(:, 1) = (H_traslation);
H_rotation_init(:, 1) =  get_rotation_dual(H_dual);
end

