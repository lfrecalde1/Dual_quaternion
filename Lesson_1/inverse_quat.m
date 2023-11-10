function [output] = inverse_quat(q)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
q_hat = congujate_quaternion(q);

norma_q =  norm_quaternion(q);

output = q_hat/(norma_q)^2;
end

