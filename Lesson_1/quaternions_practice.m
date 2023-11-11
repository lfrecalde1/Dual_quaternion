%% Quaternions Operations
clc,clear all, close all;

% Quaternions Definitions
q1 = [5; 28; 86; 99];
q2 = [-2; 0; 0; 25];


% Addition Quaternions
suma = q1+ q2
resta = q1 - q2

% Mutlplication Quaternions
multi = quaternion_multiply(q1, q2)

conjugate_aux = congujate_quaternion(q1)

norm_quat = norm_quaternion(q1)

inverse_q1 =  inverse_quat(q1)

multi_aux = quaternion_multiply(q1, inverse_q1)


