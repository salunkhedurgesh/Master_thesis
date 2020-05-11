% function name: constraints()
% Description: The calculation of the constraints of the mechanism at a
% certain configuration :checking for the constraints for only Universal 
% and Spherical joint
% Inputs: 
% 1. type of the mechanism
% 2. parameters (A (1xn) vector) 
% 3. limit for the joints
% 4. current configuration: alpha, beta
% 5. the symbolic jacobian 
% Outpus: 
% 1. Determinant of the jacobian for a given configuration
% 2. [1 x 2] boolean vector for passive universal joint of both legs
% 3. [1 x 2] boolean vector for passive spherical joint of both legs

function [det_val, p_lim_uni, p_lim_sph1, p_lim_sph2, rho_inst, collision_dist, conditioning_num] = constraints(type, parameters, limits, alpha, beta, reward)
    
    if type == "2UPS"
        [det_val, p_lim_uni, p_lim_sph1, p_lim_sph2, rho_inst, collision_dist, conditioning_num] = constraints_UPS(parameters, limits, alpha, beta, reward);
    elseif type == "2PUS"
        [det_val, p_lim_uni, p_lim_sph1, p_lim_sph2, rho_inst, collision_dist, conditioning_num] = constraints_PUS(parameters, limits, alpha, beta, reward);
    end
end