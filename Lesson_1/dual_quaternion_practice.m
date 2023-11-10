%% Code Dual Quaternions
clc, clear all, close all;
%% Possible way to write dual quaternios
h1p = [5;6;7;8];
h1d = [9;15;7;8];
h1_dual = [h1p;h1d];

h2p = [-25;0;0;1];
h2d = [0;0;7;8];
h2_dual = [h2p; h2d];


%% Sum and Subtraction
h = sum_dual(h1_dual, h2_dual);
h_rest = sub_dual(h1_dual, h2_dual);

%% multiplication based on operations
h_multiply = mult_dual(h1_dual, h2_dual);
h_multiply_aux = mult_dual(h2_dual, h1_dual);


%% Conjugte of the dual quaternion
h1_conjugate = conjugate_dual(h1_dual);

%% Norm of the dual quaternion
norm_h1 = norm_dual(h1_dual);

%% Matrix form dual quaternion
H1 = mult_dual_matrix(h1_dual);

%% Multiplication dual quaternion
h_multiply_1 = H1*h2_dual;