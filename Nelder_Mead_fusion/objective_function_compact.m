% function name: objective_function_compact()
% Description: The calculation of the evaluation of the feasible workspace
% with parameter weightage
% Inputs:
% 1. output of objective_function_workspace()
% Outpus:
% 1. evaluation of the objective function for given set of parameters
% 2. optimised actuator range

function [c_qual,rho_vec] = objective_function_compact(type, parameters, limits)
    
    [c_qual,rho_vec] = objective_function_workspace(type, parameters, limits);
    
    influencer = 12;
    
    % This is a very important variable. It decides how much does the
    % parameters and their biases affect the workspace
    % Higher the number, less is the influence of parameters on the
    % objective function. It becomes important so that a compact mechanism
    % with low feasible workspace is not given importance over a relatively
    % larger mechanism with better workspace feasibility
    
    a1 = norm(parameters(1:3), 2);
    a_prime1 = norm(parameters(4:6), 2);
    a2 = norm(parameters(7:9), 2);
    a_prime2 = norm(parameters(10:12), 2);
    t = parameters(13);
    
    if type == "2UPS"
        parameter_weight = ((a1 + a_prime1)*(t)*(a1/a_prime1)) + ((a2 + a_prime2)*(t)*(a2/a_prime2)) + influencer;
    elseif type == "2PUS"
        offset = parameters(5);
        warning("Attention!! Check the parameter assignment in objective_function_compact.m");
        parameter_weight = ((offset + a_prime)*(a)) + influencer;
    else
        fprintf("Invalid type, treating the mechanism as ''2UPS''\n");
        parameter_weight = ((a1 + a_prime1)*(t)*(a1/a_prime1)) + ((a2 + a_prime2)*(t)*(a2/a_prime2)) + influencer;
    end
    
    c_qual = c_qual/parameter_weight;
end