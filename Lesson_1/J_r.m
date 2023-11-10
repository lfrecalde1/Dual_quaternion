function [J] = J_r(theta_1, theta_2, theta_3)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
J = [ -sin(theta_1/2 + theta_2/2 + theta_3/2)/2, -sin(theta_1/2 + theta_2/2 + theta_3/2)/2, -sin(theta_1/2 + theta_2/2 + theta_3/2)/2;...
                                             0,                                         0,                                         0;...
                                             0,                                         0,                                         0;...
      cos(theta_1/2 + theta_2/2 + theta_3/2)/2,  cos(theta_1/2 + theta_2/2 + theta_3/2)/2,  cos(theta_1/2 + theta_2/2 + theta_3/2)/2];
 
end

