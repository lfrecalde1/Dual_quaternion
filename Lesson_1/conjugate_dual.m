function [h] = conjugate_dual(h1)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
h1p = h1(1:4);
h1d = h1(5:8);

h1p_conjugate = congujate_quaternion(h1p);
h1d_conjugate = congujate_quaternion(h1d);

h = [h1p_conjugate; h1d_conjugate];
end

