% function name: self_nm_input
% Description: The information required for the definition of the optimisation
% Inputs: None
% Outpus: 
% 1. Dimension of the points in optimisation
% 2. The matrix related to the ranges of the parameters
% 3. The number of starts for multi-starting the Nelder Mead process
% 4. Number of iterations to be continued in case of encountering the same solution
% 5. The vector for passive limits of U-joint, S-joint and the ratio for
% the actuator stroke. 
% 6. The choice of objective function
% 7. Availability of the sobolset function (to generate low discrepancy points)
% 8. filenames where the data is stored for the optimisation process

function [type, n, ranges, starts, iterations, limits, objective_choice, sobol_var, save_files] = self_nm_input()
    
    type = "2UPS";
    n = 4;
    ranges = [0.25, 1.5; 0.25, 2; -0.5, 0.5; 1, 4];
    starts = 2;
    iterations = 1;
    limits = [45, 45, 1.5];
    objective_choice = "compact";
    sobol_var = 0;
    save_files = ["deep.txt", "points.txt"];
    
    fprintf('The mechanism is of type %s with dimension %d\n.The number of starts are %d\n', type, n, starts);
    fprintf('The number of iterations to continue for same solution are %d \n', iterations);
    fprintf('Universal limit = %d degrees, Spherical limits = %d degrees, ratio of actuator stroke = %d\n', limits(1), limits(2), limits(3));
    fprintf('The mechnaism is being optimised for %s\n', objective_choice);
    if sobol_var == 0
        fprintf('Sobolset functionality is not used \n');
    else
        fprintf('Sobolset functionality is used to generate low discrepancy points for multi start \n');
    end
    fprintf('The file names are %s and %s. Both are opened in write mode', save_files(1), save_files(2));
end