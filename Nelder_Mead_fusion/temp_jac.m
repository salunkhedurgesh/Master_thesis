J = eye(3);
clear all
close all

parameters = [1, 2];
angles = [deg2rad(18), deg2rad(33)];
l1 = parameters(1);
l2 = parameters(2);
theta1 = angles(1);
theta2 = angles(2);
j11 = -l1 * sin(theta1) - l2 * sin(theta1 + theta2)
j12 =  - l2 * sin(theta1 + theta2)
j21 = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
j22 = l2 * cos(theta1 + theta2);
J(3,3) = 3;