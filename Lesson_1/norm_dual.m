function  n = norm_dual(h1)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
h1_conjugate = conjugate_dual(h1);
normsquare =  mult_dual(h1,h1_conjugate);
n = [sqrt(normsquare(1)); normsquare(5)/(2*sqrt(normsquare(1)))];
end

