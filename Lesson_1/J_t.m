function [J] = J_t(theta_1, theta_2, theta_3, l_1, l_2, l_3)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
J = [                                                                                      0,                                                                   0,                                     0;...
     - l_2*sin(theta_1 + theta_2) - l_1*sin(theta_1) - l_3*sin(theta_1 + theta_2 + theta_3), - l_2*sin(theta_1 + theta_2) - l_3*sin(theta_1 + theta_2 + theta_3), -l_3*sin(theta_1 + theta_2 + theta_3);...
       l_2*cos(theta_1 + theta_2) + l_1*cos(theta_1) + l_3*cos(theta_1 + theta_2 + theta_3),   l_2*cos(theta_1 + theta_2) + l_3*cos(theta_1 + theta_2 + theta_3),  l_3*cos(theta_1 + theta_2 + theta_3);...
                                                                                          0,                                                                   0,                                     0];
 
end

