% function name: determinant_UPS()
% Description: Calculates the determinant and saves its symbolic expression
% Inputs: None
% Outpus: symbolic determinant

clear all;
close all;
parameters = 2 * [0.961267 	-0.946255 	0.079538 	0.855752 	-1.197830 	0.154155 	0.947608 	0.251929 	0.092246 	0.890766 	0.453128 	0.274530 	3.853361];

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
parameters(13) = t;
alpha = deg2rad(18);
beta = deg2rad(33);
[rho1_range, rho2_range] = rho_range_UPS(parameters);
rho1_stroke = rho1_range(2) - rho1_range(1);
rho2_stroke = rho2_range(2) - rho2_range(1);

j11 = (2*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*(s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha)) + 2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*(s12y*cos(alpha) - s12z*cos(beta)*sin(alpha) + s12x*sin(beta)*sin(alpha)))/(2*(abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))^2)^(1/2));
j12 = -(2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*(s12x*cos(beta)*cos(alpha) + s12z*sin(beta)*cos(alpha)) - 2*abs(s12x*cos(beta) - u11x + s12z*sin(beta))*sign(s12x*cos(beta) - u11x + s12z*sin(beta))*(s12z*cos(beta) - s12x*sin(beta)) + 2*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*(s12x*cos(beta)*sin(alpha) + s12z*sin(beta)*sin(alpha)))/(2*(abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))^2)^(1/2));
j21 = (2*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*(s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha)) + 2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*(s22y*cos(alpha) - s22z*cos(beta)*sin(alpha) + s22x*sin(beta)*sin(alpha)))/(2*(abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))^2)^(1/2));
j22 = -(2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*(s22x*cos(beta)*cos(alpha) + s22z*sin(beta)*cos(alpha)) - 2*abs(s22x*cos(beta) - u21x + s22z*sin(beta))*sign(s22x*cos(beta) - u21x + s22z*sin(beta))*(s22z*cos(beta) - s22x*sin(beta)) + 2*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*(s22x*cos(beta)*sin(alpha) + s22z*sin(beta)*sin(alpha)))/(2*(abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))^2)^(1/2));

jac_mat = [j11, j12; j21, j22];

vaf_jac = jac_mat' * jac_mat;
eigens = eig(vaf_jac);
normalized_eigen(1,1) = max(eigens)/max(rho1_stroke, rho2_stroke);
normalized_eigen(2,1) = min(eigens)/min(rho1_stroke, rho2_stroke);
