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

function [det_var, p_lim_uni, p_lim_sph, rho_inst] = constraints(type, parameters, limits, alpha, beta)
    
    if type == "2UPS"
        [det_var, p_lim_uni, p_lim_sph, rho_inst] = constraints_UPS(parameters, limits, alpha, beta);
    elseif type == "2PUS"
        [det_var, p_lim_uni, p_lim_sph, rho_inst] = constraints_PUS(parameters, limits, alpha, beta);
    end
end