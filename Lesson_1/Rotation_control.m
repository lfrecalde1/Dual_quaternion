% (C) Copyright 2020-2023 Murilo Marques Marinho (murilomarinho@ieee.org)
%
%     This file is licensed in the terms of the
%     Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0) 
%     license.
%
% Contributors to this file:
%     Murilo Marques Marinho (murilomarinho@ieee.org)
%% Robot Control Basics using DQ Robotics - Part 2
%% Introduction
% In the last lesson, I introduced you to the basics of kinematic modeling and 
% kinematic control using a 1-DoF planar robot. The main point of that lesson 
% was to teach you how to develop your robots from scratch, if needed.
% 
% Nonetheless, the DQ Robotics library has many of those functionalities 
% built-in. In this lesson, you will learn how to model serial manipulators using 
% the Denavit-Hartenberg parameters and how to calculate important Jacobians using 
% DQ Robotics. You will also learn how to create a basic kinematic controller 
% using DQ Robotics.
%% Notation
% Keep these in mind (we will also use this notation when writting papers to 
% conferences and journals):
% 
% * ${\it \bf h} \in\mathbb{H}$: a quaternion. (Bold-face, lowercase character)
% * $\underline{{\it \bf h}}\in\mathcal{H}$: a dual quaternion. (Bold-face, 
% underlined, lowercase character)
% * ${\it \bf{p}},{\it \bf{t}},\cdots\in \mathbb{H}_p$: pure quaternions. They 
% represent points, positions, and translations. They are quaternions for which 
% $\text{Re}\left({\it \bf h}\right)=0$.
% * ${\it \bf r}\in\mathbb{S}^3$: unit quaternions. They represent orientations 
% and rotations. They are quaternions for which $||{\it \bf h}||=1$. 
% * $\underline{\it \bf x} \in$$\underline{\mathcal{S}}$: unit dual quaternions. 
% They represent poses and pose transformations. They are dual quaternions for 
% which $||{\it \bf h}||=1$.
% * $\underline{{\it \bf l}}\in \mathcal{H}_p\cap$$\underline{\bf{\mathcal{S}}}$: 
% a Plücker line. 
% * $\underline{{\it \bf \pi}}\in \left\{ P\left(\underline{{\it \bf \pi}}\right) 
% \in \mathbb{H}_p\right\}\cap\underline{\bf{\mathcal{S}}}$: a plane.
%% Problem Definition
% 
% 
% # Let the robot $R$ be a 3-DoF planar robot, as drawn in Fig.1. 
% # Let $\mathcal{F}_W$ be the world-reference frame. 
% # Let $\underline{\it \bf{x}}^W_R(t)\triangleq\underline{\it \bf{x}}_R$ $\in$$\underline{\mathcal{S}}$ 
% represent the pose of the _end effector_. 
% # Let $R$ be composed of three rotational joints that rotates about their 
% z-axis, composed in the joint-space vector ${\it \bf q}\left(t\right)\triangleq{\it 
% \bf q}=\left[\theta_{1}\ \theta_{2}\ \theta_{3}\right]^{T}$ with $\theta_{1}\left(t\right)\triangleq\theta_{1},\theta_{2}\left(t\right)\triangleq\theta_{2},\theta_{3}\left(t\right)\triangleq\theta_{3}\in\mathbb{R}$. 
% The rotation of the reference frames of each joint coincide with the rotation 
% of $\mathcal{F}_W$ when $\theta_{1}=\theta_{2}=\theta_{3}=0$. The length of 
% the links are $l_1 ,l_2 ,l_3 \in$$\mathbb{R}^+-\{0\}$.
% # Consider that we can freely control the joint vector ${\it \bf q}$.
% 
% Problems: 
% 
% # Obtain the (pose) foward kinematic model of the robot $R$ using a set of 
% DH parameters.
% # Obtain the pose Jacobian, rotation Jacobian, and translation Jacobian of 
% $R$.
% # Using 1. and 2., design a closed-loop pose controller, rotation controller, 
% and translation controller.
%% Modeling serial robots using Denavit-Hartenberg Parameters
% In the last lesson, you modelled a 2-DoF planar robot. As the number of DoF 
% and the complexity of the robots increase, modeling them requires a more general, 
% systematic, and scalable strategy. In this lesson we will show how serial manipulators 
% are modeled using the Denavit-Hartenberg (DH) parameters. This is the standard 
% methodology used in DQ robotics for modeling serial robots. 
%% Forward Kinematic Model using DH parameters
% For robots in 3D space, obtaining the robot's pose transformation is the most 
% generic form of FKM for the end effector. When using unit dual quaternions, 
% retrieving the rotation, translation, etc from the pose is quite straighforward. 
% So let us obtain the pose FKM of the robot $R$ using the DH parameters.
% 
% Before going into detail about the DH parameters, let $\underline{{\it 
% \bf x}}^W_{0}\in\underline{{\bf \mathcal{S}}}$ be the reference frame at the 
% base of the robot. For convenience, it can coincide with the reference frame 
% of the first joint of the robot. 
% 
% The first joint enacts a pose transformation from the reference frame of 
% the first joint to the reference frame of the second joint given by
% 
% $$\underline{{\it \bf x}}_{1}^{0}\left(\theta_{1}\right)\triangleq\underline{{\it 
% \bf x}}_{1}^{0}\in\underline{{\bf \mathcal{S}}}$$
% 
% that depends on the joint value of the first joint. 
% 
% Given that the 3-DoF planar robot has three joints, the robot can be modeled 
% with three consecutive transformations
% 
% $$\underline{{\it \bf x}}_{R}=\underline{{\it \bf x}}_{1}^{0}\underline{{\it 
% \bf x}}_{2}^{1}\underline{{\it \bf x}}_{3}^{2},$$
% 
% where $\underline{{\it \bf x}}_{2}^{1}\left(\theta_{2}\right)\triangleq\underline{{\it 
% \bf x}}_{2}^{1}\in\underline{{\bf \mathcal{S}}}$ and $\underline{{\it \bf x}}_{3}^{2}\left(\theta_{3}\right)\triangleq\underline{{\it 
% \bf x}}_{3}^{2}\in\underline{{\bf \mathcal{S}}}$. This sequence of transformation 
% is a methodology that can be used for a serial manipulator with any number of 
% joints.
% 
% The DH parameters provide a systematic way to calculate each individual 
% joint transformation of any n-DoF serial manipulator.  Each joint transformation, 
% $\underline{{\it \bf x}}_{i}^{i-1}\left(\theta_{i}\right)\triangleq\underline{{\it 
% \bf x}}_{i}^{i-1}\in\underline{{\bf \mathcal{S}}}$, with $i=1,2,3\ldotp \ldotp 
% \ldotp n$ is composed of four intermediate transformations, as follows
% 
% $$\underline{{\it \bf x}}_{i}^{i-1}\triangleq\underline{{\it \bf x}}_{i'}^{i}\left(\theta_{i}\right)\underline{{\it 
% \bf x}}_{i''}^{i'}\left(d_{i}\right)\underline{{\it \bf x}}_{i'''}^{i''}\left(a_{i}\right)\underline{{\it 
% \bf x}}_{i}^{i'''}\left(\alpha_{i}\right),$$
% 
% where the DH parameters, for each joint, are $\theta_{i},d_{i},a_{i},\alpha_{i}\in\mathbb{R}$. 
% Each of those parameters is related to one transformation. The first is the 
% rotation of $\theta_i$ about the z-axis of frame $\mathcal{F}_{i-1}$
% 
% $$\underline{{\it \bf x}}_{i'}^{i-1}\left(\theta_{i}\right)=\cos\left(\frac{\theta_{i}}{2}\right)+\hat{k}\sin\left(\frac{\theta_{i}}{2}\right),$$
% 
% the second is a translation of $d_i$ about the z-axis of frame $\mathcal{F}_{i'}$,
% 
% $$\underline{{\it \bf x}}_{i''}^{i'}\left(d_{i}\right)=1+\varepsilon\frac{1}{2}\hat{k}d_{i},$$
% 
% the third is the translation of $a_i$ about the x-axis of frame $\mathcal{F}_{i''}$,
% 
% $$\underline{{\it \bf x}}_{i'''}^{i''}\left(a_{i}\right)=1+\varepsilon\frac{1}{2}\hat{\imath}a_{i},$$
% 
% the fourth, and last, is the rotation of $\alpha_i$ about the x-axis of 
% frame $\mathcal{F}_{i'''}$
% 
% $$\underline{{\it \bf x}}_{i}^{i'''}\left(\alpha_{i}\right)=\cos\left(\frac{\alpha_{i}}{2}\right)+\hat{\imath}\sin\left(\frac{\alpha_{i}}{2}\right).$$
% 
% Back to our example, the following table summarizes the DH parameters of 
% the 3-DoF planar robot.
% 
% 
%% DQ Robotics Example
% Let us create a class representing the 3-DoF planar robot using DH parameters. 
% The good news is that most of the hard work is handled by DQ Robotics, using 
% the following class
%%
help DQ_SerialManipulatorDH
%% 
%  Let us represent our robot in the following way, for $l_1=l_2=l_3=1$. 
%%
% 
%   classdef ThreeDofPlanarRobotDH
%       %ThreeDofPlanarRobot regarding all methods related to the 3-DoF planar robot
%       
%       methods (Static)
%           function ret = kinematics()
%               %kinematics returns the kinematics of the ThreeDoFPlanarRobot as DQ_SerialManipulatorDH
%               DH_theta=  [0, 0, 0];
%               DH_d =     [0, 0, 0];
%               DH_a =     [1, 1, 1];
%               DH_alpha = [0, 0, 0];
%               DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,3);
%               DH_matrix = [DH_theta;
%                   DH_d;
%                   DH_a;
%                   DH_alpha;
%                   DH_type];
%               
%               ret = DQ_SerialManipulatorDH(DH_matrix,'standard');
%               ret.name = "3 DoF Planar Robot";
%           end
%       end
%   end
%
% 
% Note that we use 

DQ_SerialManipulatorDH.JOINT_ROTATIONAL;
%% 
% to define a rotational joint, so we do not explicilty define $\theta_1,\theta_2,\theta_3$ 
% in our model.  
% 
% To calculate the forward kinematics model and plot the robot model, we 
% can simply call the methods already available in the class, as follows

clear all;
close all;

% Initial joint values [rad]
theta1 = -0.1;
theta2 = 0.55;
theta3 = -1.02;

% Joint vector
q = [theta1 theta2 theta3];

% Instantiate the robot kinematics
three_dof_planar_robot = ThreeDofPlanarRobotDH.kinematics();

% Plot the robot in the xy-plane
three_dof_planar_robot.plot(q);

% Get the fkm, based on theta
x_r = three_dof_planar_robot.fkm(q);
%% 
% For more details about the methods, check the documentation of the class 
% using

help DQ_SerialManipulatorDH.fkm
help DQ_SerialManipulatorDH.plot
%% Differential Kinematics Model using DH parameters
% Similar to how we calculated the the DKM in the last lesson, the DKM can be 
% calculated for any set of DH parameters. 
%% Pose Jacobian
% Using the FKM, we find
% 
% $$\text{vec}_{8}\left(\dot{\underline{{\it \bf x}}}_{R}\right)=\frac{\partial\left(\text{vec}_{8}\left({\underline{{\it 
% \bf x}}}_{R}\right)\right)}{\partial{\it \bf q}}\dot{\it {\bf q}},$$
% 
% where ${\it \bf J}_{{\bf \underline{x}}}\triangleq\frac{\partial\left(\text{vec}_{8}\left({\underline{{\it 
% \bf x}}}_{R}\right)\right)}{\partial{\it \bf q}}$ is the pose Jacobian. We do 
% not need to worry about the details of how to calculate this for now. The details 
% of how to calculate the Jacobian for any serial manipulator are described from 
% Page 38 of
% 
% ADORNO, B. V., Two-arm manipulation: from manipulators to enhanced human-robot 
% collaboration [_Contribution à la manipulation à deux bras : des manipulateurs 
% à la collaboration homme-robot_], Université Montpellier 2, Montpellier, France, 
% 2011. (<https://adorno.eng.ufmg.br/publications/phd_thesis_final_version.pdf 
% pdf>) 
%% Rotation Jacobian
% The goal of this section is to find the rotation Jacobian, ${\it \bf J}_{r}$, 
% so that the following relation holds
% 
% $$\text{vec}_{4}\left(\dot{{\it \bf r}}_{R}\right)={\it \bf J}_{r}\dot{{\it 
% \bf q}}.$$
% 
% The rotation Jacobian is useful to control the rotation of the end effector 
% and can be used to calculate many other Jacobians.
% 
% We can conveniently find the rotation Jacobian using the pose Jacobian. 
% To do so, remember that the robot's end-effector pose can be decomposed as follows
% 
% $$\underline{{\it \bf x}}_{R}={\it \bf r}_{R}+\frac{1}{2}\varepsilon{\it 
% \bf t}_{R}{\it \bf r}_{R}.$$
% 
% That means that the first-order time-derivative is 
% 
% $$\text{vec}_{8}\left(\dot{\underline{{\it \bf x}}}_{R}\right)=\text{vec}_{8}\left(P\left(\dot{\underline{{\it 
% \bf x}}}_{R}\right)\right)+\text{vec}_{8}\left(D\left(\dot{\underline{{\it \bf 
% x}}}_{R}\right)\right)$$
% 
% that can be re-written as 
% 
% $${\it \bf J}_{{\bf \underline{x}}}\dot{{\it \bf q}}=\left[\matrix{ {\it 
% \bf J}_{P\left({\bf \underline{x}}\right)} \cr 0} \right]\dot{{\it \bf q}}+\left[\matrix{ 
% 0 \cr {\it \bf J}_{D\left({\bf \underline{x}}\right)}} \right]\dot{{\it \bf 
% q}}$$
% 
% which means that the pose Jacobian can be decomposed as
% 
% $${\it \bf J}_{{\bf \underline{x}}}\dot{{\it \bf q}}=\left[\matrix{ {\it 
% \bf J}_{P\left({\bf \underline{x}}\right)} \cr {\it \bf J}_{D\left({\bf \underline{x}}\right)}} 
% \right]\dot{{\it \bf q}}.$$
% 
% Notice that 
% 
% $${\it \bf J}_{P\left({\bf \underline{x}}\right)}\dot{{\it \bf q}}=\text{vec}_{4}\left(\dot{{\it 
% \bf r}}_{R}\right)$$
% 
% $${\it \bf J}_{P\left({\bf \underline{x}}\right)}\dot{{\it \bf q}}={\it 
% \bf J}_{r}\dot{{\it \bf q}}$$
% 
% which means that the rotation Jacobian is  ${\it \bf J}_{r}={\it \bf J}_{P\left({\bf 
% \underline{x}}\right)}$. That is, the rotational Jacobian is composed of the 
% first four rows of the pose Jacobian.
%% Translation Jacobian
% The goal of this section is to find the translation Jacobian, ${\it \bf J}_{t}$, 
% so that the following relation holds
% 
% $$\text{vec}_{4}\left(\dot{\it {\bf t}}_{R}\right)={\it \bf J}_{t}\dot{{\it 
% \bf q}}$$
% 
% The translation Jacobian is useful to control the translation of the end 
% effector and can be used to calculate many other Jacobians. We can conveniently 
% find the translation Jacobian using the pose Jacobian and the end-effector's 
% pose.
% 
% We start from the translation relation
% 
% $${\it \bf t}_{R}=\text{translation}\left(\underline{{\it \bf x}}_{R}\right)=2D\left(\underline{{\it 
% \bf x}}_{R}\right)P\left(\underline{{\it \bf x}}_{R}\right)^{*}$$
% 
% $$\dot{{\it \bf t}}_{R}=2\left[D\left(\dot{\underline{{\it \bf x}}}_{R}\right)P\left(\underline{{\it 
% \bf x}}_{R}\right)^{*}+D\left(\underline{{\it \bf x}}_{R}\right)P\left(\dot{\underline{{\it 
% \bf x}}}_{R}\right)^{*}\right]$$
% 
% $$\text{vec}_{4}\left(\dot{{\it \bf t}}_{R}\right)=2\left[\overset{-}{{\bf 
% H}}_{4}\left(P\left(\underline{{\it \bf x}}_{R}\right)^{*}\right){\it \bf J}_{D\left({\bf 
% \underline{x}}\right)}+\overset{+}{{\bf H}}_{4}\left(D\left(\underline{\it {\bf 
% x}}_{R}\right)\right){\bf C}_{4}{\it \bf J}_{P\left({\bf \underline{x}}\right)}\right]\dot{{\it 
% \bf q}}$$
% 
% hence
% 
% $${\it \bf J}_{t}=2\left[\overset{-}{{\bf H}}_{4}\left(P\left(\underline{{\it 
% \bf x}}_{R}\right)^{*}\right){\it \bf J}_{D\left({\bf \underline{x}}\right)}+\overset{+}{{\bf 
% H}}_{4}\left(D\left(\underline{{\it \bf x}}_{R}\right)\right){\bf C}_{4}{\it 
% \bf J}_{P\left({\bf \underline{x}}\right)}\right].$$
%% DQ Robotics Example
% The pose Jacobian can be computed using the DQ_SerialManipulatorDH class as 
% follows.
%%
% Get the pose Jacobian
Jx = three_dof_planar_robot.pose_jacobian(q)
%% 
% The rotation Jacobian and translation Jacobian can be calculated using 
% methods of its DQ_Kinematics superclass. For instance, the rotation Jacobian 
% can be obtained as

% Get the rotation Jacobian, based on the pose Jacobian
Jr = three_dof_planar_robot.rotation_jacobian(Jx)
%% 
% and the translation Jacobian can be obtained as

% Get the end-effector's pose
x = three_dof_planar_robot.fkm(q);
% Get the translation Jacobian
Jt = three_dof_planar_robot.translation_jacobian(Jx,x)
%% Task-space position control 
% In the last lesson you were introduced to the basics of robot control using 
% the inverse differential kinematics model. 
% 
% Instead of building the controller from scratch, you can use controllers 
% already available in DQ Robotics.
%% Pseudo-Inverse Controller
% In the last lesson, we implemented a simple pseudo-inverse-based kinematic 
% controller. Let us revisit this topic using DQ Robotics. 
% 
% Let us start by cleaning up the workspace and write the control loop from 
% scratch. 
%% Preliminaries
% Let us start with the initial conditions of the problem.
% 
% First, clean the workspace.
%%
clear all;
close all;
include_namespace_dq
%% 
% Define the sampling time and how many seconds of control we will simulate.

% Sampling time [s]
tau = 0.01; 
% Simulation time [s]
final_time = 1;
%% 
% Define the initial robot posture.

% Initial joint values [rad]
theta1 = -0.4;
theta2 = 1.71;
theta3 = 0.85;
% Arrange the joint values in a column vector
q_init = [theta1 theta2 theta3]';
%% 
% Define the desired translation

% Desired translation components [m]
tx = 1.25;
ty = 1.25;
% Desired translation
td = tx*i_ + ty*j_;
%% 
% then, the desired rotation.

% Desired rotation component [rad]
gamma = pi;
% Desired rotation
rd = cos(gamma/2.0) + k_*sin(gamma/2.0);
%% 
% The desired pose will then be

% Desired pose
xd = rd + 0.5*E_*td*rd;
%% 
% We then instantiate the robot kinematics, as follows.

% Create robot
three_dof_planar_robot = ThreeDofPlanarRobotDH.kinematics();
%% Translation Controller
% The basic syntax to instantiate a translation controller is as follows. 
% 
% The robot will align the end-effector translation with the desired translation. 
% The rotation is not controlled.
%%
% Instanteate the controller 
translation_controller = DQ_PseudoinverseController(three_dof_planar_robot);
translation_controller.set_control_objective(ControlObjective.Translation);
translation_controller.set_gain(5.0);
translation_controller.set_damping(0); % Damping was not explained yet, set it as 0 to use pinv()

% Translation controller loop.
q = q_init;
for time=0:tau:final_time
    % Get the next control signal [rad/s]
    u = translation_controller.compute_setpoint_control_signal(q,vec4(td));
    
    % Move the robot
    q = q + u*tau;
    
    
    % Plot
    % Plot the robot
    three_dof_planar_robot.plot(q);
    title(['Translation control' ' time=' num2str(time) 's out of ' num2str(final_time) 's'])
    % Plot the desired pose
    hold on
    plot(xd);
    hold off
    % [For animations only]
    drawnow; % [For animations only] Ask MATLAB to draw the plot now
end
% Rerun controller
 
%% Rotation Controller
% The basic syntax to instantiate a rotation controller is as follows.
% 
% The robot will align the end-effector rotation with the desired rotation. 
% The translation is not controlled.
%%
% Instantiate the controller 
translation_controller = DQ_PseudoinverseController(three_dof_planar_robot);
translation_controller.set_control_objective(ControlObjective.Rotation);
translation_controller.set_gain(5.0);
translation_controller.set_damping(0); % Damping was not explained yet, set it as 0 to use pinv()

% Rotation controller loop.
q = q_init;
for time=0:tau:final_time
    % Get the next control signal [rad/s]
    u = translation_controller.compute_setpoint_control_signal(q,vec4(rd));
    
    % Move the robot
    q = q + u*tau;    
    
    % Plot
    % Plot the robot
    three_dof_planar_robot.plot(q);
    title(['Rotation control' ' time=' num2str(time) 's out of ' num2str(final_time) 's'])
    % Plot the desired pose
    hold on
    plot(xd);
    hold off
    % [For animations only]
    drawnow; % [For animations only] Ask MATLAB to draw the plot now
end
% Rerun controller
 
%% Pose Controller
% The basic syntax to instantiate a pose controller is as follows. 
% 
% The robot will align the end-effector pose with the desired pose. If the 
% rotation and translation cannot be achieved simulatenously, the robot will balance 
% rotation and translation error, according to the controller definitions.
%%
% Instantiate the controller 
translation_controller = DQ_PseudoinverseController(three_dof_planar_robot);
translation_controller.set_control_objective(ControlObjective.Pose);
translation_controller.set_gain(5.0);
translation_controller.set_damping(0); % Damping was not explained yet, set it as 0 to use pinv()

% Translation controller loop.
q = q_init;
for time=0:tau:final_time
    % Get the next control signal [rad/s]
    u = translation_controller.compute_setpoint_control_signal(q,vec8(xd));
    
    % Move the robot
    q = q + u*tau;    
    
    % Plot
    % Plot the robot
    three_dof_planar_robot.plot(q);
    title(['Pose control' ' time=' num2str(time) 's out of ' num2str(final_time) 's'])
    % Plot the desired pose
    hold on
    plot(xd);
    hold off
    % [For animations only]
    drawnow; % [For animations only] Ask MATLAB to draw the plot now
end
% Rerun controller
 
%% Homework
% (1) Following the format of [ThreeDofPlanarRobotDH.m], create a class called 
% [NDofPlanarRobotDH.m] as shown in the figure. 
% 
% 
% 
% The class must
% 
% # Consider all lengths of the links as unitary. That is, $l_{1}=l_2=\cdots=l_{n}=1$. 
% # Have a "kinematics" method that takes the desired number of DoFs as input 
% and returns the corresponding DQ_SerialManipulatorDH instance.
% 
% (2) Consider the desired translation ${\it \bf t}_d=5\hat{\imath}+2\hat{\jmath}$, 
% desired rotation ${\it \bf r}_d=\cos\left(-\frac{\pi}{8}\right)+\hat{k}\sin\left(-\frac{\pi}{8}\right)$, 
% and initial posture $\theta_i(0)=\frac{\pi}{8}$ for $ i=1...7$. Use the class 
% you created in (1) to instantiate a 7-DoF planar robot.
% 
% # create a script called [seven_dof_planar_robot_translation_control.m] that 
% implements a task-space translation controller using a DQ_PseudoinverseController. 
% Control the 7-DoF planar robot to ${\it \bf t}_{d}.$
% # create a script called [seven_dof_planar_robot_rotation_control.m] that 
% implements a task-space rotation controller using a DQ_PseudoinverseController. 
% Control the 7-DoF planar robot to ${\it \bf r}_{d}.$
% # create a script called [seven_dof_planar_robot_pose_control.m] that implements 
% a task-space pose controller using a DQ_PseudoinverseController. Control the 
% 7-DoF planar robot to $\underline{{\it \bf x}}_{d}={\it \bf r}_{d}+\frac{1}{2}\varepsilon{\it 
% \bf t}_{d}{\it \bf r}_{d}$.