% HOLONOMIC_BASE_CONTROL() runs a simple example to test the DQ_MobileBase 
% class. Given an initial pose, control the robot motion towards a desired pose
% by using a switched controller in order to prevent the unwinding problem.
%
% HOLONOMIC_BASE_CONTROL(x,y,phi) enables to define the desired (x,y,phi)
% configuration.


% (C) Copyright 2019 DQ Robotics Developers
%
% This file is part of DQ Robotics.
%
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
%
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
%
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br
clc, clear all, close all;
% Robot's initial configuration
q = [0,0,0]';

% Declare a new holonomic base
holonomic_base = DQ_HolonomicBase();
holonomic_base.set_base_diameter(2); % The base diameter is set to 2m 

% Prepare the plot
figure;
% Simple heuristic to determine the workspace size

% plot the desired pose
qd = [5;5;0];
xd = holonomic_base.fkm(qd);
plot(xd);

% plot the robot in the initial pose
plot(holonomic_base,q);

%% Controller parameters
T = 0.001; % Sampling period used in the integration
gain = 50; % The greater the gain, the faster is the convergence rate.

%% Start the motion control
title('Starting motion control in three seconds...');
pause(3);
title('Go!');

% Initialize the variable used in the stop criterion
x_error = 10;
while norm(x_error) > 0.001    

    x = holonomic_base.fkm(q);
    J = holonomic_base.pose_jacobian(q);
    N = -haminus8(xd)*DQ.C8*J;
    
    % Calculate the left-invariant error for the both hemispheres
    x_error_positive = vec8(x'*xd - 1);
    x_error_negative = vec8(x'*xd + 1);
    
    % Take the smaller error
    if norm(x_error_positive) <= norm(x_error_negative)
       x_error = x_error_positive;
    else
       x_error = x_error_negative;
    end
    
    % Generate the control input
    u = pinv(N)*gain*x_error;
    % First order integration to update the configuration
    q = q + T*u;    
   
    plot(holonomic_base,q);
    drawnow;
end