function  quaternion_to_angle(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
S = [q(1); q(2); q(3); q(4)];
 
log_S = log(S)

theta = 2 * norm(imag(log_S))
end

