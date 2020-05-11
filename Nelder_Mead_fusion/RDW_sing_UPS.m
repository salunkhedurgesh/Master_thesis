% Description: check if there lies a singularity in the Regular Dextrous
%Workspace (RDW) for UPS type mechanism
% Inputs:
% 1. parameters
% None
% Outpus:
% 1. boolean for non singular RDW
function sing_bool = RDW_sing_UPS(parameters)

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

sing_bool = 1; %initializing the value to 1 by assuming that there is no singularity

for alpha = -1 : 0.01 :1
    for beta = -1 :0.01 :1
        %Singularity check
        jac_det = ((2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*(s22y*cos(alpha) - s22z*cos(beta)*sin(alpha) + s22x*sin(alpha)*sin(beta)) + 2*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*(s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta)))*(2*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*(s12x*cos(beta)*sin(alpha) + s12z*sin(alpha)*sin(beta)) + 2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*(s12x*cos(alpha)*cos(beta) + s12z*cos(alpha)*sin(beta)) - 2*abs(s12x*cos(beta) - u11x + s12z*sin(beta))*sign(s12x*cos(beta) - u11x + s12z*sin(beta))*(s12z*cos(beta) - s12x*sin(beta))))/(4*(abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))^2)^(1/2)*(abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))^2)^(1/2)) - ((2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*(s12y*cos(alpha) - s12z*cos(beta)*sin(alpha) + s12x*sin(alpha)*sin(beta)) + 2*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*(s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta)))*(2*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*(s22x*cos(beta)*sin(alpha) + s22z*sin(alpha)*sin(beta)) + 2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*(s22x*cos(alpha)*cos(beta) + s22z*cos(alpha)*sin(beta)) - 2*abs(s22x*cos(beta) - u21x + s22z*sin(beta))*sign(s22x*cos(beta) - u21x + s22z*sin(beta))*(s22z*cos(beta) - s22x*sin(beta))))/(4*(abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))^2)^(1/2)*(abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))^2)^(1/2));
        if jac_det < 0.005
            sing_bool = 0;
            break; %If any instance(alpha,beta) is singular, then the parameter is not useful
        end
    end
end
end
