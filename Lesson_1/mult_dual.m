function [h] = mult_dual(h1,h2)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
h1p = h1(1:4);
h1d = h1(5:8);

h2p = h2(1:4);
h2d = h2(5:8);

aux_p = quaternion_multiply(h1p, h2p);
aux_d = quaternion_multiply(h1p, h2d) + quaternion_multiply(h1d, h2p);

h = [aux_p; aux_d];
end

