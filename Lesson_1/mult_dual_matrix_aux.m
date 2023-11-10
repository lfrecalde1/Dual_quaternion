function [H] = mult_dual_matrix_aux(h1)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
h1p = h1(1:4);
h1d = h1(5:8);

H = [matrix_q_aux(h1p), zeros(4,4);...
    matrix_q_aux(h1d), matrix_q_aux(h1p)];

end