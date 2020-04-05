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
 if input(prompt_para) == 0
     [Best_point_yet, ball_halfwidth] = Nelder_mead_actuator_loop(sobol_var,steps,starts);
 else
     [Best_point_yet, ball_halfwidth] = Nelder_mead_para_weight(sobol_var,steps,starts);
 end
 
 fileID = fopen('Points_record_para_5_April.txt', 'a');
 fprintf('\nThe point chosen to refine is : a = %f, a_prime = %f, h = %f and t = %f \n\n', Best_point_yet(1), Best_point_yet(2), Best_point_yet(3), Best_point_yet(4));
 %Researching the surroundings of the Best_point with lower epsilon value
 [Best_point_yet, Best_rho] = Nelder_mead_refine(Best_point_yet);
 fprintf('\nThe point after refining is : a = %f, a_prime = %f, h = %f and t = %f \n\n', Best_point_yet(1), Best_point_yet(2), Best_point_yet(3), Best_point_yet(4));
 plot_code(Best_point_yet);
 plot_points(Best_point_yet, Best_rho, ball_halfwidth);
 print('Point_plot','-dpng','-r600')
 
 %Number of choice
 tempp = fopen('temp.txt', 'r');
 formatSpec = '%f %f %f %f';
 sizeA = [4,Inf];
 A = fscanf(tempp,formatSpec,sizeA);
 input_1 = 1;
 while input_1 >0
 prompt = 'Which point is to be plotted?';
 input_i = input(prompt);
 if input_i == 0
     break
 end
 S_i = A(:,input_i);
 close all;
 plot_code(S_i);
 [~,rho_vec]= obj_func_act(S_i(1), S_i(2), S_i(3), S_i(4),ball_halfwidth);
 plot_points(S_i, rho_vec, ball_halfwidth);
 end
 fclose(tempp);
 
 
 
 
