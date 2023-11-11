function [s] = vec3_skew(q)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
s = [0, -q(3), q(2);...
     q(3), 0, -q(1);...
     -q(2), q(1), 0];
end

