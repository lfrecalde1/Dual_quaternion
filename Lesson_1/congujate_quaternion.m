function [output] = congujate_quaternion(q)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%% Split Vectors
a1 = q(1);
b1 = -q(2);
c1 = -q(3);
d1 = -q(4);


%% Compute conjugate
output = [a1;b1;c1;d1];
end

