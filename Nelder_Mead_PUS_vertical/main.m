%%Main code for getting the complete results
%1. It may take time to get results as the time depends on:
    %a. The dimension of the simplex: defined as "n" in
    %Nelder_mead_actuator_loop.m
    %b. Starting simplex at each start (if valid_points > 5)
    %c. Number of starts: defined as "(valid_points/5)" in
    %Nelder_mead_actuator_loop.m
    %d. The number of iterations to stop if the same solution is
    %encountered: defined as "steps" in Nelder_mead_actuator_loop.m
%2. At the end of execution the output will be:
    %a. Records.txt: Complete data on the optimised points
    %b. Points_records.txt: Recorded optimised points of each start
    %c. Figures of the optimised configuration in following poses:  
        %c.1: alpha, beta = [0,0], [1,1], [-1,-1], [-1,1], [1,-1]
 
 clear all;
 close all;
 clc

 n = 5; %number of dimensions
 ranges = [0.5,2;0.1,1;-0.5,0.5;1,4;0.5,1.5];
%  a_range = [0.25, 1.25];
%  a_prime_range = [0.25,2];
%  h_range = [-0.5,1.5];
%  t_range = [1,4];
%  offset_range = [0.1,0.5];
 
 %Prompt to make sure the files are saved uniquely
 prompt_file_name = 'File name for deep result (eg: ''deep_xyz.txt'')';
 prompt_file_name_points = 'File name for file saving all local optimums (eg: ''points_xyz.txt'')';
 prompt_file_name_refine = 'File name for file saving the refined report (eg: ''refine_xyz.txt'')';
 
 deep_file_name = input(prompt_file_name);
 points_file_name = input(prompt_file_name_points);
 refine_file_name = input(prompt_file_name_refine);
 
 %Deleting the temp file that saves the result from all starts (For plotting purposes)
 filet = fopen('temp.txt','r');
 if filet > 0
     fclose(filet); 
     delete temp.txt;
 end
 
 %Prompts for different utilities
 prompt_sobol = 'Do you have sobolset functionality, 1 for yes and 0 for no\n';
 prompt_steps = 'How many steps do you want the same solution to continue, Recommendation: 10\n';
 prompt_starts = 'How many starts do you want, 1 for single start, Recommendation:10(Will take time)\n';
 prompt_para = 'Do you want the compactness in objective function, 1 for yes and 0 for no\n';
 
 sobol_var = input(prompt_sobol);
 steps = input(prompt_steps);
 starts = input(prompt_starts);
 %1 -> Single start optimisation, will give a local minima
 %Recommendation = 10
 %switch to 1 if you have Matlab 2017 and further with Machine Learning
 %toolbox
 %If prompt_para = 1, consider parameter weightage
            %If = 0, do not consider parameter weightage
            
 %Switch for considering the parameter weightage
 if input(prompt_para) == 0
     [Best_point_yet, ball_halfwidth] = Nelder_mead_actuator_loop(n,sobol_var,steps,starts,deep_file_name,points_file_name,ranges);
 else
     [Best_point_yet, ball_halfwidth] = Nelder_mead_para_weight(n,sobol_var,steps,starts,deep_file_name,points_file_name,ranges);
 end
 
 deep_fileID = fopen(points_file_name, 'a');
 points_fileID = fopen(points_file_name, 'a');
 fprintf('\nThe point chosen to refine is : a = %f, a_prime = %f, h = %f, t = %f offset = %f \n\n', Best_point_yet(1), Best_point_yet(2), Best_point_yet(3), Best_point_yet(4));
  
 %Researching the surroundings of the Best_point with lower epsilon value
 [Best_point_yet, Best_rho] = Nelder_mead_refine(n,Best_point_yet,refine_file_name, ranges);
 fprintf('The point after refining is:\n');
 fprintf('a = %f, a_prime = %f, h = %f, t = %f and offset = %f \n', Best_point_yet(1), Best_point_yet(2), Best_point_yet(3), Best_point_yet(4));
 fprintf('Minimum stroke length = %f and maximum stroke length = %f \n', min(Best_rho), max(Best_rho));
 fprintf(deep_fileID, 'The point after refining is:\n');
 fprintf(deep_fileID, 'a = %f, a_prime = %f, h = %f, t = %f and offset = %f \n', Best_point_yet(1), Best_point_yet(2), Best_point_yet(3), Best_point_yet(4));
 fprintf(deep_fileID, 'Minimum stroke length = %f and maximum stroke length = %f \n', min(Best_rho), max(Best_rho));
 fprintf(points_fileID, 'The point after refining is:\n');
 fprintf(points_fileID, 'a = %f, a_prime = %f, h = %f, t = %f and offset = %f \n', Best_point_yet(1), Best_point_yet(2), Best_point_yet(3), Best_point_yet(4), Best_point_yet(5));
 fprintf(points_fileID, 'Minimum stroke length = %f and maximum stroke length = %f \n', min(Best_rho), max(Best_rho));
 fclose(deep_fileID);
 fclose(points_fileID);
 
 %Plotting
 plot_code(Best_point_yet);
 plot_points(Best_point_yet, Best_rho, ball_halfwidth);
 picid = fopen('Point_plot.png', 'r');
 if picid > 0
     fclose(picid);
     delete Point_plot.png
 end
 fprintf('Saving the most recent and optimised result as Point_plot.png \n');
 print('Point_plot','-dpng','-r600')
 
 %Number of choice
 if starts > 1
     tempp = fopen('temp.txt', 'r');
     formatSpec = '%f %f %f %f %f';
     sizeA = [5,Inf];
     A = fscanf(tempp,formatSpec,sizeA);
     input_1 = 1;
     while input_1 >0
         fprintf('Which point is to be plotted from 1 to %d \n', starts);
     prompt = '(0 to stop)?\n';
     input_i = input(prompt);
     if input_i == 0
         break
     end
     S_i = A(:,input_i);
     close all;
     plot_code(S_i);
     [~,rho_vec]= obj_func_act(S_i(1), S_i(2), S_i(3), S_i(4),S_i(5),ball_halfwidth);
     plot_points(S_i, rho_vec, ball_halfwidth);
     end
     fclose(tempp);
 end
 
 
 
 
