%% Control Rigid Body Kinematics only
clc, clear all, close all;

%% Time definition
ts = 0.01;
t_final = 20;
t = (0:ts:t_final);

%% Initial Position dual quaternion formulation
t_0 = [0;1.25;1.25;0];
angle_0 = pi/4; %% problems in pi "I do not the answer for that...."

%% The rotation is similiar a rotation vector formation
r_o = rotation_quaternion(angle_0, [0;0;1]);
h_o =  pose_dual(t_0,r_o)

%% Velocities quaternions
p1_dot = 0*ones(length(t));
p2_dot = 0*ones(length(t));
p3_dot = 0*ones(length(t));

w1 = 0*ones(length(t));
w2 = 0*ones(length(t));
w3 = 0*ones(length(t));

for k = 1length(t)
    
   xi(:, k) =  
    
end