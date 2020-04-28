% Description: Calculates the actuator ranges over the desired RDW for UPS
% Inputs:
% 1. parameters
% None
% Outpus:
% 1. the maximum and minimum actuator lengths over the span of desired RDW

function [rho1_range, rho2_range] = rho_range_UPS(parameters)
    
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
    s12_mobile = [s12x; s12y; s12z];
    s22_mobile = [s22x; s22y; s22z];
    r_i = 1;
    for alpha = -1:0.01:1
        for beta = -1:0.01:1
            fixed_T_mobile = trans_mat('z', t)*rot_mat('x', alpha)*rot_mat('y', beta);
            s12 = fixed_T_mobile*[s12_mobile;1];
            s22 = fixed_T_mobile*[s22_mobile;1];
            
            u11_T_fixed = trans_mat('x', -u11x)*trans_mat('y', -u11y)*trans_mat('z', -u11z);
            u21_T_fixed = trans_mat('x', -u21x)*trans_mat('y', -u21y)*trans_mat('z', -u21z);
            
            s12_u11 = u11_T_fixed*s12;
            s22_u21 = u21_T_fixed*s22;

            rho1_vec(r_i) = norm(s12_u11(1:3),2);
            rho2_vec(r_i) = norm(s22_u21(1:3),2);
            
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

