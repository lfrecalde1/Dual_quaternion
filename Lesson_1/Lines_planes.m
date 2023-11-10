%% Dual quaternions Lines, planes and more
clc, clear all, close all;

%% Line direction
l1 = [0;1;0;0];
pl1 = [0;0;0;0];
m = cross_quaternion(pl1, l1)