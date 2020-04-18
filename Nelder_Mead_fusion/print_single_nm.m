% function name: print_single_nm()
% Description: Saving the filenames for further process
% Inputs:
% None
% Outpus:
% 1. Prints the necessary information on screen
% 2. saves the same information on a file for record of deep analysis

function print_single_nm(deep_fileID, single_best_point, single_best_rho, single_eval, iterations, optimum)
    
    n = length(single_best_point);
    if optimum == 1
        fprintf("\nThe optimised point is found \n");
        for k = 1:n
            fprintf("\nParameter %d = %f, \t", k, single_best_point(k));
        end
        fprintf("\nThe actuator ranges are: ");
        fprintf("%f \t", single_best_rho);
        fprintf("\nThe evaluation at this point is %d \n \n", single_eval);
        
        fprintf(deep_fileID, "\nThe optimised point is found \n");
        for k = 1:n
            fprintf(deep_fileID, "\nParameter %d = %f, \t", k, single_best_point(k));
        end
        fprintf(deep_fileID, "\nThe actuator ranges are: ");
        fprintf(deep_fileID, "%f \t", single_best_rho);
        fprintf(deep_fileID, "\nThe evaluation at this point is %d \n \n", single_eval);
        
    else
        fprintf("\nThe simplex stopped after %d iterations \n", iterations);
        for k = 1:n
            fprintf("\nParameter %d = %f, \t", k, single_best_point(k));
        end
        fprintf("\nThe actuator ranges are: ");
        fprintf("%f \t", single_best_rho);
        fprintf("\nThe evaluation at this point is %d \n \n", single_eval);
        
        fprintf(deep_fileID, "\nThe simplex stopped after %d iterations \n", iterations);
        for k = 1:n
            fprintf(deep_fileID, "\nParameter %d = %f, \t", k, single_best_point(k));
        end
        fprintf(deep_fileID, "\nThe actuator ranges are: ");
        fprintf(deep_fileID, "%f \t", single_best_rho);
        fprintf(deep_fileID, "\nThe evaluation at this point is %d \n \n", single_eval);
    end
end