% This is an attempt to write a general Nelder Mead code
% To change a mechanism please take care of the following files:
% 1. constraints.m: the inverse kinematics of the mechanism you want to optimize
% 2. determinant.m : The symbolic expression of the Jacobian 
% 3. RDW_sing: the parameters defined should be present with determinant.m
% 4. rho_range: It is a sub-part of the inverse kinematics where the actuator ranges are required

close all;
clear all;
clc

warning('Please check the file names properly because the files are opened in writing mode and not in appending mode');
prompt_you = 'Do you want to enter the inputs (0 for no and 1 for yes)\n';
if input(prompt_you) == 0
     [type, n, ranges, starts, iterations, limits, objective_choice, sobol_var, save_files] = self_nm_input();
else
    [type, n, ranges, starts, iterations, limits, objective_choice, sobol_var] = NMinput();
    
    [save_files] = filenames();
end

[best_point, best_rho] = nelder_mead_ms(type, n, ranges, starts, iterations, limits, objective_choice, sobol_var, save_files);
