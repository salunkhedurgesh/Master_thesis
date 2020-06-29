% function name: objective_function_compact()
% Description: The calculation of the evaluation of the feasible workspace
% with parameter weightage
% Inputs:
% 1. output of objective_function_workspace()
% Outpus:
% 1. evaluation of the objective function for given set of parameters
% 2. optimised actuator range

function [c_qual,rho_vec] = objective_function_compact(type, parameters, limits)
    
    reward = "binary";
    maximize = "workspace";
    [c_qual,rho_vec] = objective_function_live(type, parameters, limits, reward, maximize);
    actuator_mean = (rho_vec(1) + rho_vec(2))/2;
    influencer = 12;
    
    % This is a very important variable. It decides how much does the
    % parameters and their biases affect the workspace
    % Higher the number, less is the influence of parameters on the
    % objective function. It becomes important so that a compact mechanism
    % with low feasible workspace is not given importance over a relatively
    % larger mechanism with better workspace feasibility
    
    a1 = parameters(1);
    a_prime1 = parameters(4);
    a2 = parameters(7);
    a_prime2 = parameters(10);
    t = parameters(13);
    a1p = [a1*cos(parameters(2)), a1*sin(parameters(2))];
    a_prime1p = [a_prime1*cos(parameters(5)), a_prime1*sin(parameters(5))];
    a2p = [a2*cos(parameters(8)), a2*sin(parameters(8))];
    a_prime2p = [a_prime2*cos(parameters(11)), a_prime2*sin(parameters(11))];
    triangle1_side = norm((a1p-a2p),2);
    triangle2_side = norm((a_prime1p-a_prime2p),2);
    s1 = (a1 + a2 + triangle1_side)/2;
    s2 = (a_prime1 + a_prime2 + triangle2_side)/2;
    area1 = sqrt(s1*(s1-a1)*(s1-a2)*(s1-triangle1_side));
    area2 = sqrt(s2*(s2-a_prime1)*(s2-a_prime2)*(s2-triangle2_side));
    mean_area = (area1 + area2)/2;
    
    if type == "2UPS"
%         parameter_weight = ((a1 + a_prime1)*(t)*(a1/a_prime1)) + ((a2 + a_prime2)*(t)*(a2/a_prime2)) + influencer;
        parameter_weight = mean_area * t;
    elseif type == "2PUS"
        warning("Attention!! Check the parameter assignment in objective_function_compact.m \n");
        parameter_weight = ((offset + a_prime)*(a)) + influencer;
    else
        fprintf("Invalid type, treating the mechanism as ''2UPS''\n");
%         parameter_weight = ((a1 + a_prime1)*(t)*(a1/a_prime1)) + ((a2 + a_prime2)*(t)*(a2/a_prime2)) + influencer;
        parameter_weight = mean_area * t;
    end
    
    if c_qual > 0.75*40401
        c_qual = c_qual*(1 + parameter_weight);
    end
end