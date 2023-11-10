function [output] = norm_quaternion(q)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
h1 = q;
h_c = congujate_quaternion(q);

product = quaternion_multiply(h1, h_c);

output = sqrt(product);
output = output(1);
end

