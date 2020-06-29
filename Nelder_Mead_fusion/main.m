% This is an attempt to write a general Nelder Mead code
% To change a mechanism please take care of the following files:
% 1. constraints.m: the inverse kinematics of the mechanism you want to optimize
% 2. determinant.m : The symbolic expression of the Jacobian 
% 3. RDW_sing: the parameters defined should be present with determinant.m
% 4. rho_range: It is a sub-part of the inverse kinematics where the actuator ranges are required

close all;
clear all;
clc

start_time = clock;
prompt_you = 'Do you want to enter the inputs (0 for no and 1 for yes)\n';
if input(prompt_you) == 0
    [type, n, ranges, starts, iterations, limits, objective_choice, sobol_var, save_files, reward, maximize] = self_nm_input();
else
    [type, n, ranges, starts, iterations, limits, objective_choice, sobol_var, reward, maximize] = NMinput();
    
    [save_files] = filenames();
end
format shortg
c = clock;
foldername = num2str(c(3)) + "_" + num2str(c(2));
if ~exist(foldername, 'dir')
    mkdir (foldername);
end

save_files(1) = foldername + "/" + save_files(1);
save_files(2) = foldername + "/" + save_files(2);

f1 = fopen(save_files(1),'r');
f2 = fopen(save_files(2),'r');
if f1 > 0
    fclose(f1);
    error('File %s already exists, change name to avoid data loss\n', save_files(1));
elseif f2 > 0
    fclose(f2);
    error('File %s already exists, change name to avoid data loss\n', save_files(2));
end

prompt_git = 'Type the necessary update - Why are you running the code? \n';
git_mood = input(prompt_git);

[best_point, best_rho] = nelder_mead_ms(type, n, ranges, starts, iterations, limits, objective_choice, sobol_var, save_files, reward, maximize, git_mood);
end_time = clock;

plot_code(best_point);
plot_valid(type, best_point, limits, best_rho);

elapsed_time = etime(end_time, start_time);
thours = floor(elapsed_time/3600);
temp1 = mod(elapsed_time, 3600);
tminutes = floor(temp1/60);
tseconds = mod(temp1, 60);

fprintf("The total time elapsed for comple operation is %d hours %d minutes and %.1f seconds \n", thours, tminutes, tseconds);
reopen_points_fileID = fopen(save_files(2),'a');
fprintf(reopen_points_fileID, "The total time elapsed for comple operation is %d hours %d minutes and %.1f seconds\n", thours, tminutes, tseconds);
fclose(reopen_points_fileID);