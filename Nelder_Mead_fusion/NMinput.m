% function name: NMinput
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

function [type, n, ranges, starts, iterations, limits, objective_choice, sobol_var, reward] = NMinput()
    
    % Defining the configuration of the mechanism 
    prompt_type = 'Input the type of mechanism you want to optimise\n';
    fprintf('Options:\n1 -> 2UPS \n2 -> 2PUS\n');
    type = input(prompt_type);
    if type == 1
        type = "2UPS";
    elseif type == 2
        type = "2PUS";
    else
        error("Invalid input, please enter either 1 or 2");
    end
    
    % Defining the dimension of the problem. It means the number of
    % parameters that are to be optimised, eg:[a, a_prime, h, t] suggests
    % that n = 4 in 2UPS+1U and in 2PUS+1U we have n = 5; [a, a_prime, h, t, offset]
    prompt_dimension = 'Dimension of the optimisation problem (n)\n';
    n = input(prompt_dimension);
    if n < 1
        error('Optimisation dimension should be a positive integer');
    end
    
    % Defining the ranges of the parameters to search points in the
    % optimisation model. The dimension should be n x 2. It contains the
    % lower bound and upper bound of each parameter
    fprintf('Input the range of %d parameters in a %d x 2 matrix', n, n);
    prompt_ranges = '\n';
    ranges = input(prompt_ranges);
    if size(ranges(1,:)) ~= 2
        error('Wrong dimension of ranges matrix, the number of coluns should be 2');
    elseif size(ranges(:,1)) ~= n
        error('Wrong dimension of ranges matrix, the number of rows should be equal to the dimension of the optimisation (n)');
    end
    for i = 1:n
        if ranges(i,1) >= ranges(i,2)
            fprintf('Error in defining range for parameter %d\n', i);
            fprintf('lower bound = %f and upper bound = %f\n', ranges(i,1), ranges(i,2));
            error('Error in defining ranges: the lower bound of atleast one of the parameters is either equal to or greater than upper bound');
        end
    end
    
    % Defining the number of starts for the Nelder MEad optimisation. The
    % multi-start is necessary as te Nelder Mead often returns a local
    % minima. In order to achieve global minima, we have to start the
    % Simplec process with several different initial points.
    prompt_start = 'Number of multi-starts for the optimisation problem\n';
    starts = input(prompt_start);
    if mod(starts, 1) ~= 0
        fprintf('WARNING: The input for starts is %d, it will be changed to %d\n', starts, floor(starts));
        starts = floor(starts);
    elseif starts < 1
        error('Incorrect number of starts. The input should be a positive integer\n');
    end
    
    % Defining the number of iterations to be continued in case of
    % encountering the same solution. This is used to save time. Ideally
    % the Simplex should converger so small that distance between any two
    % parameters of the Simplex point is less than a small epsilon value. 
    prompt_iterations = 'Number of iterations in case of non-changing solution point (Recommended value: 10)\n';
    iterations = input(prompt_iterations);
    
    % Defining the limits:
    % 1. Passive joint limit for Universal joint
    % 2. Passive joint limit for Spherical joint
    % Ratio for the stroke length and size of the actuator.
    % The stroke (rho1 and rho2) varies from a minimum length to
    % ratio*minimum length. This helps us optimizing the mechnaism with
    % feasible actuator ranges
    prompt_uni_lim = 'The angle limit for Universal joint in degrees\n';
    prompt_sph_lim = 'The angle limit for Spherical joint in degrees\n';
    prompt_stroke_lim = 'The ratio of extended:closed position of actuator is (eg: 1.5)\n';    
    limits = [input(prompt_uni_lim), input(prompt_sph_lim), input(prompt_stroke_lim)];
    
    % Defining the type of objective function we want
    % The options are:
    % 1. workspace: Searches for a design that provides the maximum
    % workspace
    % 2. compact: Searches for a design with highest workspace:size ratio 
    
    prompt_objective_choice = 'What type of objective function are you looking for \n';
    fprintf('The options being:\n');
    fprintf('1 -> ''workspace'' : Searches for a design that provides the maximum workspace\n');
    fprintf('2 -> ''compact'' : Searches for a design with highest workspace:size ratio \n');
    objective_choice = input(prompt_objective_choice);
    if objective_choice == 2
        fprintf('NOTE: Please check for the parameter weightage in file: \n');
        fprintf('objective_function_compact.m\n');
    end
    if objective_choice == 1
        objective_choice = "workspace";
    elseif objective_choice == 2
        objective_choice = "compact"; 
    else
        error('Incorrect input for the choice of the objective function, please input either 1 or 2 \n');
    end
    
    % Taking input for the sobolset functionality
    prompt_sobol = 'Do you have sobolset functionality (1 for yes, 0 for no)\n';
    sobol_var = input(prompt_sobol);
    if sobol_var == 0 || sobol_var == 1
        fprintf("\n");
    else
        fprintf('WARNING: Invalid input, treating as 0\n');
        sobol_var = 0;
    end
    
    % Taking input for the reward type
    prompt_reward = 'Type of reward function("binary", "linear", "invert")\n';
    reward = input(prompt_reward);
    if reward == "binary" || reward == "linear" || reward == "invert"
        fprintf("\n");
    else
        fprintf('WARNING: Invalid input, treating as binary reward\n');
        reward = "binary";
    end
    
    % Taking input for the maximizing other objective function
    prompt_multimax = 'Type of maximize function("joint_quality", "conditioning_number")\n';
    multimax = input(prompt_multimax);
    if multimax == "joint_quality" || multimax == "conditioning_number"
        fprintf("\n");
    else
        fprintf('WARNING: Invalid input, treating as joint_quality multi-objective function\n');
        reward = "joint_quality";
    end
    
end