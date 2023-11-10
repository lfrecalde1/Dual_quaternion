function [quaternion] = rotation_quaternion(angle, axis)
%UNTITLED14 Summary of this function goes here
%   Detailed explanation goes here
real_part  = cos(angle/2);
imaginary_part = axis*sin(angle/2);

quaternion = [real_part; imaginary_part(1); imaginary_part(2); imaginary_part(3)];
end

