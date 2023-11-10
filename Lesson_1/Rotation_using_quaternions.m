%% Quaternions example with rotations
clc, clear all, close all;


angle_1 = pi/8;
angle_2 = pi/16;
angle_3 = pi/16;
r1 = rotation_quaternion(angle_1, [1;0;0])

norm_r1 = norm_quaternion(r1)

r2 = rotation_quaternion(angle_2,  [1; 0; 0])
norm_r2 = norm_quaternion(r2)

r3 = rotation_quaternion(angle_3,  [0; 1; 0])
norm_r3 = norm_quaternion(r3)

r4_aux = quaternion_multiply(r1, r2);

r4 = quaternion_multiply(r4_aux, r3)

r1_inv = congujate_quaternion(r1)

r1_unit = quaternion_multiply(r1, r1_inv)
