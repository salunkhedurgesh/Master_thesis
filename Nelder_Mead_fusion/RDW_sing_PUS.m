% Description: check if there lies a singularity in the Regular Dextrous
%Workspace (RDW) for PUS type mechanism
% Inputs:
% 1. parameters
% None
% Outpus:
% 1. boolean for non singular RDW
function sing_bool = RDW_sing_PUS(parameters)
    
    a = parameters(1);
    a_prime = parameters(2);
    h = parameters(3);
    t = parameters(4);
    offset = parameters(5);
    
    sing_bool = 1; %initializing the value to 1 by assuming that there is no singularity
    
    for alpha = -1 : 0.01 :1
        for beta = -1 :0.01 :1
            %Singularity check
            jac_det = ((2*h^2*cos(beta)*sin(beta) - 2*h*sin(alpha)*sin(beta)*(offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha)))/(2*(- (offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 - h^2*sin(beta)^2 + a^2)^(1/2)) + h*cos(alpha)*sin(beta))*(((h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))*(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta)))/(- (h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))^2 - (- offset + a_prime*cos(beta) + h*sin(beta))^2 + a^2)^(1/2) + h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta)) - (((2*h*cos(beta) - 2*a_prime*sin(beta))*(a_prime*cos(beta) - offset + h*sin(beta)) - (h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))*(2*a_prime*cos(beta)*sin(alpha) + 2*h*sin(alpha)*sin(beta)))/(2*(- (h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))^2 - (- offset + a_prime*cos(beta) + h*sin(beta))^2 + a^2)^(1/2)) + a_prime*cos(alpha)*cos(beta) + h*cos(alpha)*sin(beta))*(h*cos(beta)*sin(alpha) - a_prime*cos(alpha) + ((a_prime*sin(alpha) + h*cos(alpha)*cos(beta))*(offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha)))/(- (offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 - h^2*sin(beta)^2 + a^2)^(1/2));
            if jac_det < 0.005
                sing_bool = 0;
                break; %If any instance(alpha,beta) is singular, then the parameter is not useful
            end
        end
    end
end
