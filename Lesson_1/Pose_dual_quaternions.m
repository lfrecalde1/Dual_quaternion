%% Dual quaternion traslations and rotations
clc, clear all, close all;

%% Definition first transformation
%% the position only has values in i j k
t1 = [0;0;0;0];
angle_1 = 0;

%% The rotation is similiar a rotation vector formation
r1 = rotation_quaternion(angle_1, [1;0;0]);

%% Traslation and then rotation
h1_dual = pose_dual(t1,r1)

%% the position only has values in i j k
t2 = [0;0;1;0];
angle_2 = pi/2;

%% The rotation is similiar a rotation vector formation
r2 = rotation_quaternion(angle_2, [1;0;0]);

%% Traslation and then rotation
h2_dual = pose_dual(t2,r2)

%% the position only has values in i j k
t3 = [0;2;0;-1];
angle_3 = pi/16;

%% The rotation is similiar a rotation vector formation
r3 = rotation_quaternion(angle_3, [0;0;1]);

%% Traslation and then rotation
h3_dual = pose_dual(t3,r3)

%% Norm of the dual quaternion
h1_dual_norm = norm_dual(h1_dual)
h2_dual_norm = norm_dual(h2_dual)
h3_dual_norm = norm_dual(h3_dual)

%% Total transformation
h_total_aux = mult_dual(h1_dual, h2_dual)
h_total  = mult_dual(h_total_aux, h3_dual)

%% Conjugate of the total trasnformation
h_total_conjugate = conjugate_dual(h_total)
identity = mult_dual_matrix(h_total)*h_total_conjugate

%% Get Traslation and rotation ofthe dual quaternion
h3_traslation =  get_traslatation_dual(h3_dual)
h3_rotation =  get_rotation_dual(h3_dual)