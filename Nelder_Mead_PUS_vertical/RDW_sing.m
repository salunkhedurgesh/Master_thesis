%Function to check if there lies a singularity in the Regular Dextrous
%Workspace (RDW)
function sing_bool = RDW_sing(a, a_prime, h, t, offset)
    hk =0; %For simplicity in the beginning, I am not involving hk 
    sing_bool = 1; %initializing the value to 1 by assuming that there is no singularity
    
    for alpha = -1 : 0.01 :1
        for beta = -1 :0.01 :1
            %Singularity check
            jac_det = ((2*h^2*cos(beta)*sin(beta) - 2*h*sin(beta)*sin(alpha)*(offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha)))/(2*(- (offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 - h^2*sin(beta)^2 + a^2)^(1/2)) + h*sin(beta)*cos(alpha))*(h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha) + ((h*cos(beta)*cos(alpha) - a_prime*sin(beta)*cos(alpha))*(h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha)))/(- (a_prime*cos(beta) - offset + h*sin(beta))^2 - (h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha))^2 + a^2)^(1/2)) - (((a_prime*sin(alpha) + h*cos(beta)*cos(alpha))*(offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha)))/(- (offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 - h^2*sin(beta)^2 + a^2)^(1/2) - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*(a_prime*cos(beta)*cos(alpha) - (2*(a_prime*cos(beta)*sin(alpha) + h*sin(beta)*sin(alpha))*(h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha)) - 2*(h*cos(beta) - a_prime*sin(beta))*(a_prime*cos(beta) - offset + h*sin(beta)))/(2*(- (a_prime*cos(beta) - offset + h*sin(beta))^2 - (h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha))^2 + a^2)^(1/2)) + h*sin(beta)*cos(alpha));
            if jac_det < 0.005
                sing_bool = 0;
                break; %If any instance(alpha,beta) is singular, then the parameter is not useful
            end   
        end
    end
end
