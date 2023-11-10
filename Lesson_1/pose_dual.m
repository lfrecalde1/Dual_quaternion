function [h] = pose_dual(t1,r1)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
h = [r1;1/2*(quaternion_multiply(t1, r1))];
end

