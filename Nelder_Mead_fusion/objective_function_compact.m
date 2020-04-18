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
    influencer = 10;
    % This is a very important variable. It decides how much does the
    % parameters and their biases affect the workspace
    % Higher the number, less is the influence of parameters on the
    % objective function. It becomes important so that a compact mechanism
    % with low feasible workspace is not given importance over a relatively
    % larger mechanism with better workspace feasibility
    
    a = parameters(1);
    a_prime = parameters(2);
    h = parameters(3);
    t = parameters(4);
    
    if type == "2UPS"
        parameter_weight = ((a + a_prime)*(abs(h) + t)*(a/a_prime)) + influencer;
    elseif type == "2PUS"
        offset = parameters(5);
        parameter_weight = ((offset + a_prime)*(a)) + influencer;
    else
        fprintf("Invalid type, treating the mechanism as ''2UPS''\n");
        parameter_weight = ((a + a_prime)*(abs(h) + t)*(a/a_prime)) + influencer;
    end
    
    c_qual = c_qual/parameter_weight;
end