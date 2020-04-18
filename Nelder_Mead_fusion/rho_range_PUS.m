% Description: Calculates the actuator ranges over the desired RDW for PUS
% Inputs:
% 1. parameters
% None
% Outpus:
% 1. the maximum and minimum actuator lengths over the span of desired RDW

function [rho1_range,rho2_range] = rho_range_PUS(parameters)
    
    a = parameters(1);
    a_prime = parameters(2);
    h = parameters(3);
    t = parameters(4);
    offset = parameters(5);
    s12_mobile = [a_prime;0;h];
    s22_mobile = [0;a_prime;h];
    r_i = 1;
    for alpha = -1:0.1:1
        for beta = -1:0.1:1
            fixed_T_mobile = trans_mat('z', t)*rot_mat('x', alpha)*rot_mat('y', beta);
            s12 = fixed_T_mobile*[s12_mobile;1];
            s22 = fixed_T_mobile*[s22_mobile;1];
            
            rho1_vec(r_i) = s12(3) + sqrt(a^2 - s12(2)^2 - (offset - s12(1))^2);
            rho2_vec(r_i) = s22(3) + sqrt(a^2 - s22(1)^2 - (offset - s22(2))^2);
            
            r_i = r_i + 1;
        end
    end
    rho1_range = zeros(1,2);
    rho2_range = zeros(1,2);
    
    rho1_range(1) = min(rho1_vec);
    rho1_range(2) = max(rho1_vec);
    rho2_range(1) = min(rho2_vec);
    rho2_range(2) = max(rho2_vec);
end

