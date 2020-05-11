% function name: print_single_nm()
% Description: Saving the filenames for further process
% Inputs:
% None
% Outpus:
% 1. Prints the necessary information on screen
% 2. saves the same information on a file for record of deep analysis

function print_single_nm(deep_fileID, single_best_point, single_best_rho, single_eval, iterations, optimum, mean_iter_time, c_qual_one)
    
    n = length(single_best_point);
    if optimum == 1
        fprintf("\nThe optimised point is found \n");
        for k = 1:n
            fprintf("\nParameter %d = %f, \t", k, single_best_point(k));
        end
        fprintf("\nThe actuator ranges are: ");
        fprintf("%f \t", single_best_rho);
        fprintf("\nThe evaluation at this point is %d \n", single_eval);
        fprintf("The mean iteration time is %0.2f seconds\n", mean_iter_time);
        fprintf("The percentage of feasible workspace is %0.2f %% \n\n", abs(c_qual_one/404.01));
        
        fprintf(deep_fileID, "\nThe optimised point is found \n");
        for k = 1:n
            fprintf(deep_fileID, "\nParameter %d = %f, \t", k, single_best_point(k));
        end
        fprintf(deep_fileID, "\nThe actuator ranges are: ");
        fprintf(deep_fileID, "%f \t", single_best_rho);
        fprintf(deep_fileID, "\nThe evaluation at this point is %d \n", single_eval);
        fprintf(deep_fileID, "The mean iteration time is %0.2f seconds\n\n", mean_iter_time);
        fprintf(deep_fileID, "The percentage of feasible workspace is %0.2f %% \n\n", abs(c_qual_one/404.01));
    else
        fprintf("\nThe simplex stopped after %d iterations \n", iterations);
        for k = 1:n
            fprintf("\nParameter %d = %f, \t", k, single_best_point(k));
        end
        fprintf("\nThe actuator ranges are: ");
        fprintf("%f \t", single_best_rho);
        fprintf("\nThe evaluation at this point is %d \n", single_eval);
        fprintf("The mean iteration time is %0.2f seconds\n\n", mean_iter_time);
        fprintf("The percentage of feasible workspace is %0.2f %% \n\n", abs(c_qual_one/404.01));
        
        fprintf(deep_fileID, "\nThe simplex stopped after %d iterations \n", iterations);
        for k = 1:n
            fprintf(deep_fileID, "\nParameter %d = %f, \t", k, single_best_point(k));
        end
        fprintf(deep_fileID, "\nThe actuator ranges are: ");
        fprintf(deep_fileID, "%f \t", single_best_rho);
        fprintf(deep_fileID, "\nThe evaluation at this point is %d \n", single_eval);
        fprintf(deep_fileID, "The mean iteration time is %0.2f seconds\n\n", mean_iter_time);
        fprintf(deep_fileID, "The percentage of feasible workspace is %0.2f %% \n\n", abs(c_qual_one/404.01));
    end
end