% function name: objective_function()
% Description: The calculation of the objective function
% Inputs: 
% 1. type of the mechanism
% 2. objective_choice 
% 3. parameters (A (1xn) vector) 
% 4. limit for the joints
% Outpus: 
% 1. evaluation of the objective function for given set of parameters
% 2. optimised actuator range

function [c_qual, rho_vec] = objective_function(type, objective_choice, parameters, limits, reward, maximize)
    
    if objective_choice == "workspace"
        [c_qual,rho_vec] = objective_function_live(type, parameters, limits, reward, maximize);
%         fprintf("Live objective function in use\n");
    elseif objective_choice == "compact"
        [c_qual,rho_vec] = objective_function_compact(type, parameters, limits);
    end
end