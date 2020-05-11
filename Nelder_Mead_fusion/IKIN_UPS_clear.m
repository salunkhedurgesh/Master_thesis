% Inverse kinematics of "2UPS - 1U" with SS-joint and spherical orientation
% such that all the joints are in their default alignment (zero rotation)
% when alpha = 0 and beta = 0 (the home position of the mechanism)

% Through the 13 parameters, we know:
parameters = [0.25, 0, 0, 0.35, 0, 1, 0, 0.25, 0, 0, 0.35, 1, 1];
limits = [45, 45, 1.5];
u11x = parameters(1);
u11y= parameters(2);
u11z = parameters(3);

s12x = parameters(4);
s12y= parameters(5);
s12z = parameters(6);

u21x = parameters(7);
u21y= parameters(8);
u21z = parameters(9);

s22x = parameters(10);
s22y= parameters(11);
s22z = parameters(12);

t = parameters(13);

origin = [0; 0; 0];
in_origin_x = [1; 0; 0];
in_origin_y = [0; 1; 0];
in_origin_z = [0; 0; 1];

in_origin_t = [0; 0; t];

in_origin_u11 = [u11x; u11y; u11z];
in_origin_u21 = [u21x; u21y; u21z];

in_t_s12_initial = [s12x; s12y; s12z];
in_t_s22_initial = [s22x; s22y; s22z];

alpha = pi/4;
beta = pi/4;
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
