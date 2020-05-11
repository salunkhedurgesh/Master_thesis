% function name: constraints_UPS()
% Description: The calculation of the constraints of the mechanism at a
% certain configuration :checking for the constraints for only Universal
% and Spherical joint
% Inputs:
% 1. parameters (A (1xn) vector)
% 2. limit for the joints
% 3. current configuration: alpha, beta
% Outpus:
% 1. Determinant of the jacobian for a given configuration
% 1. [1 x 2] boolean vector for passive universal joint of both legs
% 2. [1 x 2] boolean vector for passive spherical joint of both legs

function [det_val, p_lim_uni, p_lim_sph1, p_lim_sph2, rho_inst] = constraints_UPS_weight(parameters, limits, alpha, beta)
% Inverse kinematics of "2UPS - 1U" with SS-joint and spherical orientation
% such that all the joints are in their default alignment (zero rotation)
% when alpha = 0 and beta = 0 (the home position of the mechanism)

% Through the 13 parameters, we know:
u11x = parameters(1) * cos(parameters(2));
u11y = parameters(1) * sin(parameters(2));
u11z = parameters(3);

s12x = parameters(4) * cos(parameters(5));
s12y = parameters(4) * sin(parameters(5));
s12z = parameters(6);

u21x = parameters(7) * cos(parameters(8));
u21y = parameters(7) * sin(parameters(8));
u21z = parameters(9);

s22x = parameters(10) * cos(parameters(11));
s22y = parameters(10) * sin(parameters(11));
s22z = parameters(12);

t = parameters(13);

det_val = ((2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*(s22y*cos(alpha) - s22z*cos(beta)*sin(alpha) + s22x*sin(alpha)*sin(beta)) + 2*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*(s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta)))*(2*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*(s12x*cos(beta)*sin(alpha) + s12z*sin(alpha)*sin(beta)) + 2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*(s12x*cos(alpha)*cos(beta) + s12z*cos(alpha)*sin(beta)) - 2*abs(s12x*cos(beta) - u11x + s12z*sin(beta))*sign(s12x*cos(beta) - u11x + s12z*sin(beta))*(s12z*cos(beta) - s12x*sin(beta))))/(4*(abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))^2)^(1/2)*(abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))^2)^(1/2)) - ((2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*(s12y*cos(alpha) - s12z*cos(beta)*sin(alpha) + s12x*sin(alpha)*sin(beta)) + 2*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*(s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta)))*(2*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*(s22x*cos(beta)*sin(alpha) + s22z*sin(alpha)*sin(beta)) + 2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*(s22x*cos(alpha)*cos(beta) + s22z*cos(alpha)*sin(beta)) - 2*abs(s22x*cos(beta) - u21x + s22z*sin(beta))*sign(s22x*cos(beta) - u21x + s22z*sin(beta))*(s22z*cos(beta) - s22x*sin(beta))))/(4*(abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))^2)^(1/2)*(abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))^2)^(1/2));

origin = [0; 0; 0];
in_origin_x = [1; 0; 0];
in_origin_y = [0; 1; 0];
in_origin_z = [0; 0; 1];

in_origin_t = [0; 0; t];

in_origin_u11 = [u11x; u11y; u11z];
in_origin_u21 = [u21x; u21y; u21z];

in_t_s12_initial = [s12x; s12y; s12z];
in_t_s22_initial = [s22x; s22y; s22z];

% Assuming (for representation purpose) that the pose of the end-effector
% at any particular configuration can be given by Rx(alpha) * Ry(beta)
wrt_origin_T_of_t = trans_mat('z', t) * rot_mat('x', alpha) * rot_mat('y', beta);
wrt_origin_T_of_t_initial = trans_mat('z', t);

% It is same as saying that I travel from O to t first and then as I am
% sitting on the universal joint now, it is time to rotate by 'alpha' about
% the x-axis and then rotate by 'beta' about the y-axis of rotated frame

in_origin_s12 = wrt_origin_T_of_t * [in_t_s12_initial; 1];
in_origin_s12_initial = wrt_origin_T_of_t_initial * [in_t_s12_initial; 1];

in_origin_s22 = wrt_origin_T_of_t * [in_t_s22_initial; 1];
in_origin_s22_initial = wrt_origin_T_of_t_initial * [in_t_s22_initial; 1];

[rho1, p_lim_u11, p_lim_s121, p_lim_s122] = leg_ikin(in_origin_t, in_origin_u11, in_origin_s12, in_origin_s12_initial, in_t_s12_initial, limits);
[rho2, p_lim_u21, p_lim_s221, p_lim_s222] = leg_ikin(in_origin_t, in_origin_u21, in_origin_s22, in_origin_s22_initial, in_t_s22_initial, limits);

rho_inst = [rho1, rho2];
p_lim_uni = [p_lim_u11, p_lim_u21];
p_lim_sph1 = [p_lim_s121, p_lim_s221];
p_lim_sph2 = [p_lim_s122, p_lim_s222];

end