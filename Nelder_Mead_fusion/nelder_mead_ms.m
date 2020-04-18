% function name: nelder_mead_ms()
% Description: Implementation of single start Nelder Mead
% Inputs: None
% Outpus:
% 1. type of the mechanism
% 2. initial simplex
% 3. objective_choice
% 4. ranges of the parameters
% 5. limit for the joints
% 6. sobolset functionality

function [best_point, best_rho] = nelder_mead_ms(type, n, ranges, starts, iterations, limits, objective_choice, sobol_var, save_files)
    
    fprintf('Opening file named %s for storing the deep analysis of the multi-start optimisation \n', save_files(1));
    deep_fileID = fopen(save_files(1),'w');
    fprintf(deep_fileID, 'The type of mechanism optimised is %s\n', type);
    fprintf(deep_fileID, 'The dimension of the optimisation is %d, the number of starts are %d\n', n, starts);
    fprintf(deep_fileID, 'The number of iterations if same solution is encountered is %d \n', iterations);
    fprintf(deep_fileID, 'The limits on universal joint is %d degrees, spherical joint is %d degrees and the stroke ration for actuators is %d \n', limits(1), limits(2), limits(3));
    fprintf(deep_fileID, 'The objective function aims to calculate %s design \n', objective_choice);
    fprintf(deep_fileID, 'The ranges for the parameters are: \n\n');
    for i = 1:n
        fprintf(deep_fileID, 'Parameter %d: [%0.2f %0.2f] \n', i, ranges(i,1), ranges(i,2));
    end
    
    fprintf('Please wait: Calculating the set of valid points for a initial simplex point that is non-singular in the desired RDW \n')
    
    if sobol_var == 0
        sobol_set = rand_tuning(type, ranges, starts);
    else
        sobol_set = sobol_tuning(type, ranges, starts);
    end
    
    fprintf('Done: Calculated initial simplexes for %d starts \n\n', starts);
    multi_eval = 0;
    
    for multi_start = 1:starts
        co_eff_mat = [1, 2, 0.5, 0.5]; %[reflection, expansion, contraction, shrinkgae]
        S = sobol_set(((n+1)*(multi_start-1) +1):(n+1)*multi_start,:);
        [single_best_point, single_best_rho, single_eval, optimum] = nelder_mead(type, S, iterations, objective_choice, ranges, limits, co_eff_mat);
        
        print_single_nm(deep_fileID, single_best_point, single_best_rho, single_eval, iterations, optimum);
        
        % Post single start
        saved_S_eval(multi_start,:) = [single_best_point, single_eval];
        if single_eval < multi_eval
            best_point = single_best_point;
            best_rho = single_best_rho;
            multi_eval = single_eval;
        end
        
        fprintf("Best point yet is: ");
        fprintf("%f \t", best_point);
        fprintf("\nThe actuator range is: ");
        fprintf("%f \t", best_rho);
        fprintf("\n");
        fprintf(deep_fileID, "Best point yet is: ");
        fprintf(deep_fileID, "%f \t", best_point);
        fprintf(deep_fileID, "\nThe actuator range is: ");
        fprintf(deep_fileID, "%f \t", best_rho);
        fprintf(deep_fileID, "\n");
    end
    fprintf('\nCompleted all starts of Nelder Mead: Closing file %s \n', save_files(1));
    fclose(deep_fileID);
    
    % Saving the points file
    
    fprintf('Opening a file named %s to save all the optimised points with different starting simplex \n \n', save_files(2));
    points_fileID = fopen(save_files(2),'a');
    fprintf(points_fileID, 'The type of mechanism optimised is %s\n', type);
    fprintf(points_fileID, 'The dimension of the optimisation is %d, the number of starts are %d\n', n, starts);
    fprintf(points_fileID, 'The number of iterations if same solution is encountered is %d \n', iterations);
    fprintf(points_fileID, 'The limits on universal joint is %d degrees, spherical joint is %d degrees and the stroke ration for actuators is %d \n', limits(1), limits(2), limits(3));
    fprintf(points_fileID, 'The objective function aims to calculate %s design \n', objective_choice);
    fprintf(points_fileID, 'The ranges for the parameters are: \n\n');
    for i = 1:size(saved_S_eval,1)
        fprintf(points_fileID, "\n The parameters for start %d are:\n", i);
        fprintf(points_fileID, "%f \t", saved_S_eval(i,1:n));
        fprintf(points_fileID, "\nThe evaluation for this parameters is: " );
        fprintf(points_fileID, "%f \n", saved_S_eval(i, n+1));
    end
    fprintf(points_fileID, "\nThe best point before refining is:\n");
    fprintf(points_fileID, "%f \t", best_point);
    fprintf(points_fileID, "With evaluation %f \n", multi_eval);
    fclose(points_fileID);
    
    fileID3 = fopen('temp.txt','w');
    for i = 1:size(saved_S_eval,1)
        fprintf(fileID3, "%f \t", saved_S_eval(i,1:n));
    end
    fclose(fileID3);
    
end
