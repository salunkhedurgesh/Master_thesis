% Description: Calculates the actuator ranges over the desired RDW
% Inputs:
% 1. types
% 2. parameters
% None
% Outpus:
% 1. the maximum and minimum actuator lengths over the span of desired RDW

function [rho1_range,rho2_range] = rho_range(type, parameters)
    
    if type == "2UPS"
        [rho1_range,rho2_range] = rho_range_UPS(parameters);
    elseif type == "2PUS"
        [rho1_range,rho2_range] = rho_range_PUS(parameters);
    end
end