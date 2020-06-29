
% clear cnum;
% clear all
% close all
% clc
% % parameters = [1.460027 	1.492489 	0.074953 	1.270403 	-1.592705 	0.207068 	1.406987 	-0.592608 	0.085110 	0.920942 	-0.066873 	0.410076 	3.903761];
% parameters = [1.305506 	0.175931 	0.015756 	0.931778 	-1.507154 	0.058928 	1.312649 	0.999608 	-0.059587 	0.329540 	1.420063 	0.019006 	3.986142];
% parameters = [1         -0.1            0            1         -0.1            0            1       1.4708            0    1       1.4708            0            4];
function [op1, op2] = hmap_coarse(parameters)
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
i = 1;
j = 1;
cum_val = 0;
elements = 0;
alpha_border = 0.8;
beta_border = 0.8;
for beta = 3 : -0.01 : -3
    for alpha = -3 : 0.01 : 3
        j11 = (2*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*(s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha)) + 2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*(s12y*cos(alpha) - s12z*cos(beta)*sin(alpha) + s12x*sin(beta)*sin(alpha)))/(2*(abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))^2)^(1/2));
        j12 = -(2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*(s12x*cos(beta)*cos(alpha) + s12z*sin(beta)*cos(alpha)) - 2*abs(s12x*cos(beta) - u11x + s12z*sin(beta))*sign(s12x*cos(beta) - u11x + s12z*sin(beta))*(s12z*cos(beta) - s12x*sin(beta)) + 2*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*(s12x*cos(beta)*sin(alpha) + s12z*sin(beta)*sin(alpha)))/(2*(abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))^2)^(1/2));
        j21 = (2*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*(s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha)) + 2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*(s22y*cos(alpha) - s22z*cos(beta)*sin(alpha) + s22x*sin(beta)*sin(alpha)))/(2*(abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))^2)^(1/2));
        j22 = -(2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*(s22x*cos(beta)*cos(alpha) + s22z*sin(beta)*cos(alpha)) - 2*abs(s22x*cos(beta) - u21x + s22z*sin(beta))*sign(s22x*cos(beta) - u21x + s22z*sin(beta))*(s22z*cos(beta) - s22x*sin(beta)) + 2*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*(s22x*cos(beta)*sin(alpha) + s22z*sin(beta)*sin(alpha)))/(2*(abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))^2)^(1/2));
        
        jac_mat = [j11, j12; j21, j22];
        ellipsoid_jac = jac_mat' * jac_mat; %because jac_mat mat is J^(-1)
        vaf = eig(ellipsoid_jac);
        S = svd(jac_mat);
        cnum(i, j) = min(S)/max(S);
        cnum_vaf(i,j) = min(vaf)/max(vaf);
        vaf_min(i,j) = (1 - min(vaf))^2;
        vaf_max(i,j) = (1 - max(vaf))^2;
        vaf1d = (norm([(1-min(vaf)), (1-max(vaf))]))+ 0.1;
        vmi = regularise_vaf(min(vaf));
        vma = regularise_vaf(max(vaf));
        vaf1d = norm([vmi, vma]);
        vaf1dist(i,j) = abs(max(vaf) - min(vaf))/sqrt(2);
        obj_vaf(i,j) = vaf1d;
        wobj_vaf(i,j) = vaf1d/(vaf1dist(i,j) + 1);
        if abs(alpha) <= alpha_border && abs(beta) <= beta_border
            cum_val = cum_val + cnum(i,j);
            elements = elements + 1;
            rdw_points(elements, :) = [i, j];
            rdw_angles(elements, :) = [alpha, beta];
            rdw_cnum(elements) = cnum(i,j);
            rdw_vaf(elements, :) = vaf;
        end
        j = j+1;
    end
    i = i+1;
    j = 1;
end
mean_val = cum_val/elements;
cum_variance = 0;
point_vector = zeros(1, 101);
for reloop = 1:elements
    cum_variance = cum_variance + (cnum(rdw_points(reloop,1), rdw_points(reloop, 2)) - mean_val)^2;
    temp_c = floor(rdw_cnum(reloop)*100);
    temp_in = temp_c + 1; % to avoid the zero floor
    point_vector(1, temp_in) = point_vector(1, temp_in) + 1;
end


std_devi= sqrt(cum_variance/elements);

[min_con, min_index] = min(rdw_cnum);
[max_con, max_index] = max(rdw_cnum);
[min_vafmin, vafmin_index] = min(rdw_vaf(:, 1));
[max_vafmax, vafmax_index] = max(rdw_vaf(:, 2));
% title_string = ['Heat Map for the conditioning number', ' ', num2str(min_con), ' ', num2str(max_con)];
title_string_report = 'Heat Map for the conditioning number';
h = heatmap(cnum, 'GridVisible','off', 'Colormap',parula, 'title', title_string_report);
set(gca,'FontSize',14, 'FontName', 'CMU Serif')
cdl = h.XDisplayLabels;
size(h.XDisplayLabels)
gap = zeros(1, (size(h.XDisplayLabels, 1) - 1)/12 - 1);
gap_static = repmat(" ",1, size(gap,2));
label_arrayx = [];
label_arrayy = [];
for label_i = -3:0.5:3
    if label_i == 3
        label_arrayx = [label_arrayx, '3'];
    else
        temp_str = string(label_i);
        label_arrayx = [label_arrayx, temp_str, gap_static];
    end
end

for ylabel_i = 3:-0.5:-3
    if ylabel_i == -3
        label_arrayy = [label_arrayy, '-3'];
    else
        temp_stry = string(ylabel_i);
        label_arrayy = [label_arrayy, temp_stry, gap_static];
    end
end
op1 = size(label_arrayx, 1);
op2 = size(label_arrayy, 1);

% Current Display Labels
% h.XDisplayLabels = repmat(' ',size(cdl,1), size(cdl,2));
h.XDisplayLabels = label_arrayx';
h.YDisplayLabels = label_arrayy';


title_string_reportvmin = 'Heat Map for the (1- vaf_{min})^2';
figure()
vmin_plot = heatmap(vaf_min, 'GridVisible','off', 'Colormap',summer, 'title', title_string_reportvmin);
set(gca,'FontSize',14, 'FontName', 'CMU Serif')

title_string_reportvmax = 'Heat Map for the (1 - vaf_{max})^2';
figure()
vmax_plot = heatmap(vaf_max, 'GridVisible','off', 'Colormap',spring, 'title', title_string_reportvmax);
set(gca,'FontSize',14, 'FontName', 'CMU Serif')

figure()
cnumvaf_plot = heatmap(cnum_vaf, 'GridVisible','off', 'Colormap',parula, 'title', 'cnum vaf');
set(gca,'FontSize',14, 'FontName', 'CMU Serif')
cnumvaf_plot.XDisplayLabels = label_arrayx';
cnumvaf_plot.YDisplayLabels = label_arrayy';

figure()
objvaf_plot = heatmap(obj_vaf, 'GridVisible','off', 'Colormap',parula, 'title', 'Objective function of vaf');
set(gca,'FontSize',14, 'FontName', 'CMU Serif')
objvaf_plot.XDisplayLabels = label_arrayx';
objvaf_plot.YDisplayLabels = label_arrayy';

figure()
vaf1dist_plot = heatmap(vaf1dist, 'GridVisible','off', 'Colormap',parula, 'title', 'Objective function of distance of the point vaf');
set(gca,'FontSize',14, 'FontName', 'CMU Serif')
vaf1dist_plot.XDisplayLabels = label_arrayx';
vaf1dist_plot.YDisplayLabels = label_arrayy';

figure()
wobjvaf_plot = heatmap(wobj_vaf, 'GridVisible','off', 'Colormap',parula, 'title', 'Objective function of vaf');
set(gca,'FontSize',14, 'FontName', 'CMU Serif')
wobjvaf_plot.XDisplayLabels = label_arrayx';
wobjvaf_plot.YDisplayLabels = label_arrayy';




vmin_plot.XDisplayLabels = label_arrayx';
vmin_plot.YDisplayLabels = label_arrayy';
vmax_plot.XDisplayLabels = label_arrayx';
vmax_plot.YDisplayLabels = label_arrayy';


fprintf("The standard deviation is %.2f \n", std_devi);
fprintf("The mean value is %.2f \n", mean_val);
fprintf("The minimum value is %.3f and the maximum value is %.2f \n", min_con, max_con);
fprintf("Minimum configuration at %0.2f, %0.2f \n", rdw_angles(min_index, 1), rdw_angles(min_index, 2));
fprintf("Maximum configuration at %0.2f, %0.2f \n", rdw_angles(max_index, 1), rdw_angles(max_index, 2));

fprintf("The minimum value for vaf_min is %.3f \n", min_vafmin);
fprintf("Minimum vafmin configuration at %0.2f, %0.2f \n", rdw_angles(vafmin_index, 1), rdw_angles(vafmin_index, 2));

fprintf("The maximum vafmax value is %.2f \n", max_vafmax);
fprintf("Maximum vafmax configuration at %0.2f, %0.2f \n", rdw_angles(vafmax_index, 1), rdw_angles(vafmax_index, 2));
end