function [output] = cross_quaternion(q1, q2)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
x = q1(1);
y = q1(2);
z = q1(3);
w = q1(4);

S = [0, -w, z, -y;...
     w, 0, -x, z;...
     -z, x, 0, -w;...
     y, -z, w, 0];
output = S*q2;
end

