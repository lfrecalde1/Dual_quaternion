function [h] = sum_dual(h1, h2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
h1p = h1(1:4);
h1d = h1(5:8);

h2p = h2(1:4);
h2d = h2(5:8);


sum_p = h1p + h2p;
sum_d = h1d + h2d;

h = [sum_p; sum_d];
end

