function [t] = get_traslatation_dual(h1)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
h1p = h1(1:4);
h1d = h1(5:8);

t = 2 * matrix_q_aux(h1p)'*h1d;
end

