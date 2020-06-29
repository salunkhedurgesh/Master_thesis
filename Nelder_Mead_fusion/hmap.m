 function [h] = hmap(parameters)
clear cnum;
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
for beta = 3 : -0.01 : -3
    for alpha = -3 : 0.01 : 3
        j11 = (2*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*(s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha)) + 2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*(s12y*cos(alpha) - s12z*cos(beta)*sin(alpha) + s12x*sin(beta)*sin(alpha)))/(2*(abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))^2)^(1/2));
        j12 = -(2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*(s12x*cos(beta)*cos(alpha) + s12z*sin(beta)*cos(alpha)) - 2*abs(s12x*cos(beta) - u11x + s12z*sin(beta))*sign(s12x*cos(beta) - u11x + s12z*sin(beta))*(s12z*cos(beta) - s12x*sin(beta)) + 2*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*(s12x*cos(beta)*sin(alpha) + s12z*sin(beta)*sin(alpha)))/(2*(abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))^2)^(1/2));
        j21 = (2*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*(s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha)) + 2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*(s22y*cos(alpha) - s22z*cos(beta)*sin(alpha) + s22x*sin(beta)*sin(alpha)))/(2*(abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))^2)^(1/2));
        j22 = -(2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*(s22x*cos(beta)*cos(alpha) + s22z*sin(beta)*cos(alpha)) - 2*abs(s22x*cos(beta) - u21x + s22z*sin(beta))*sign(s22x*cos(beta) - u21x + s22z*sin(beta))*(s22z*cos(beta) - s22x*sin(beta)) + 2*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*(s22x*cos(beta)*sin(alpha) + s22z*sin(beta)*sin(alpha)))/(2*(abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))^2)^(1/2));
        
        jac_mat = [j11, j12; j21, j22];
        S = svd(jac_mat);
        cnum(i, j) = min(S)/max(S);
        if abs(beta) <= 1 || abs(alpha) <= 1
            cum_val = cum_val + cnum(i,j);
            elements = elements + 1;
            rdw_points(elements, :) = [i, j];
            rdw_angles(elements, :) = [alpha, beta];
            rdw_cnum(elements) = cnum(i,j); 
        end
        j = j+1;
    end
    i = i+1;
    j = 1;
end


min_con = min(rdw_cnum);
max_con = max(rdw_cnum);
title_string = ['Heat Map for the conditioning number', ' ', num2str(min_con), ' ', num2str(max_con)];
h = heatmap(cnum, 'GridVisible','off', 'Colormap',summer, 'title', title_string);
cdl = h.XDisplayLabels;  
size(h.XDisplayLabels)
gap = zeros(1, (size(h.XDisplayLabels, 1) - 1)/12 - 1);
gap_static = repmat(" ",1, size(gap,2));
label_array = [];
for label_i = -3:0.5:3
    if label_i == 3
        label_array = [label_array, '3'];
    else        
        temp_str = string(label_i);
        temp_str_size = length(temp_str);
        
        temp_int = size(gap,2) - temp_str_size + 1;
        gap2 = repmat(' ',1, temp_int);
        label_array = [label_array, temp_str, gap_static];
    end
end

% Current Display Labels
% h.XDisplayLabels = repmat(' ',size(cdl,1), size(cdl,2));
h.YDisplayLabels = label_array';
h.XDisplayLabels = label_array';

end