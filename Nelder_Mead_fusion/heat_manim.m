parameters = [1.424194 	-1.704771 	-0.079518 	1.355703 	-0.144525 	0.173362 	0.870690 	1.626155 	-0.070115 	0.594410 	1.352876 	-0.040175 	2.953253 ];
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

for beta = 3 : -0.1 : -3
    for alpha = -3 : 0.1 : 3
        j11 = (2*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*(s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha)) + 2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*(s12y*cos(alpha) - s12z*cos(beta)*sin(alpha) + s12x*sin(beta)*sin(alpha)))/(2*(abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))^2)^(1/2));
        j12 = -(2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))*(s12x*cos(beta)*cos(alpha) + s12z*sin(beta)*cos(alpha)) - 2*abs(s12x*cos(beta) - u11x + s12z*sin(beta))*sign(s12x*cos(beta) - u11x + s12z*sin(beta))*(s12z*cos(beta) - s12x*sin(beta)) + 2*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))*(s12x*cos(beta)*sin(alpha) + s12z*sin(beta)*sin(alpha)))/(2*(abs(t - u11z + s12y*sin(alpha) + s12z*cos(beta)*cos(alpha) - s12x*sin(beta)*cos(alpha))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(beta)*sin(alpha))^2)^(1/2));
        j21 = (2*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*(s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha)) + 2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*(s22y*cos(alpha) - s22z*cos(beta)*sin(alpha) + s22x*sin(beta)*sin(alpha)))/(2*(abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))^2)^(1/2));
        j22 = -(2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))*(s22x*cos(beta)*cos(alpha) + s22z*sin(beta)*cos(alpha)) - 2*abs(s22x*cos(beta) - u21x + s22z*sin(beta))*sign(s22x*cos(beta) - u21x + s22z*sin(beta))*(s22z*cos(beta) - s22x*sin(beta)) + 2*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))*(s22x*cos(beta)*sin(alpha) + s22z*sin(beta)*sin(alpha)))/(2*(abs(t - u21z + s22y*sin(alpha) + s22z*cos(beta)*cos(alpha) - s22x*sin(beta)*cos(alpha))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(beta)*sin(alpha))^2)^(1/2));
        
        jac_mat = [j11, j12; j21, j22];
        S = svd(jac_mat);
        cnum(i, j) = min(S)/max(S);   
        j = j+1;
    end
    i = i+1;
    j = 1;
end

h = heatmap(cnum, 'GridVisible','off', 'Colormap',hot);
cdl = h.XDisplayLabels;                                    % Current Display Labels
h.XDisplayLabels = repmat(' ',size(cdl,1), size(cdl,2));  
h.YDisplayLabels = repmat(' ',size(cdl,1), size(cdl,2)); 