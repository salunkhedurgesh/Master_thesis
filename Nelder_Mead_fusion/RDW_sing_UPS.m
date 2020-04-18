% Description: check if there lies a singularity in the Regular Dextrous
%Workspace (RDW) for UPS type mechanism
% Inputs:
% 1. parameters
% None
% Outpus:
% 1. boolean for non singular RDW
function sing_bool = RDW_sing_UPS(parameters)
    
    a = parameters(1);
    a_prime = parameters(2);
    h = parameters(3);
    t = parameters(4);
    
    sing_bool = 1; %initializing the value to 1 by assuming that there is no singularity
    
    for alpha = -1 : 0.01 :1
        for beta = -1 :0.01 :1
            %Singularity check
            jac_det = ((2*abs(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))*sign(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))*(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta)) - 2*abs(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))*sign(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))*(h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta)))*(2*h*abs(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*sign(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*sin(alpha)*sin(beta) - 2*h*sign(h*sin(beta))*cos(beta)*abs(h*sin(beta)) + 2*h*abs(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))*sign(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))*cos(alpha)*sin(beta)))/(4*(abs(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))^2 + abs(a + a_prime*cos(beta) + h*sin(beta))^2 + abs(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))^2)^(1/2)*(abs(h*sin(beta))^2 + abs(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 + abs(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))^2)^(1/2)) + ((2*abs(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*sign(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*(a_prime*sin(alpha) + h*cos(alpha)*cos(beta)) + 2*abs(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))*sign(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))*(a_prime*cos(alpha) - h*cos(beta)*sin(alpha)))*(2*abs(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))*sign(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))*(a_prime*cos(alpha)*cos(beta) + h*cos(alpha)*sin(beta)) - 2*abs(a + a_prime*cos(beta) + h*sin(beta))*sign(a + a_prime*cos(beta) + h*sin(beta))*(h*cos(beta) - a_prime*sin(beta)) + 2*abs(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))*sign(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))*(a_prime*cos(beta)*sin(alpha) + h*sin(alpha)*sin(beta))))/(4*(abs(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))^2 + abs(a + a_prime*cos(beta) + h*sin(beta))^2 + abs(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))^2)^(1/2)*(abs(h*sin(beta))^2 + abs(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 + abs(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))^2)^(1/2));
            if jac_det < 0.005
                sing_bool = 0;
                break; %If any instance(alpha,beta) is singular, then the parameter is not useful
            end
        end
    end
end
