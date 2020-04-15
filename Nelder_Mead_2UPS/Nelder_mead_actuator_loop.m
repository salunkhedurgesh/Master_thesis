%%Nelder Mead implementation
%Stage 1: Choosing the simplex: Implementing sobol function for choosing
%         valid points with minimum discrepancy
%Stage 2: Stopping Criteria
%Stage 3: Evaluating the objective function and initializing the simplex
%Stage 4: Successive reflection, Expansion, contraction and Shrinkage
%Stage 5: Verifying the validity of the generated new point
%Code by: Durgesh Salunkhe, at Ecole Centrale de Nantes

function [Best_point_yet, ball_halfwidth] = Nelder_mead_actuator_loop(sobol_var,steps,starts,deep_file_name,points_file_name)
    
    fprintf('Opening file named Report.text for storing the optimised points after each start \n');
    fileID = fopen(deep_file_name,'a');
    sphere_limit = 45;
    ball_halfwidth = sind(sphere_limit);

    %% Stage 1: Choosing the Simplex

    n = 4; %number of dimensions
    fprintf('The dimension of the optimisation is %d \n', n);
    fprintf(fileID,'The dimension of the optimisation is %d \n', n);

    %First simplex of 5 points
    a_range = [0.25,1.25];
    a_prime_range = [0.25,2];
    h_range = [-0.5,1.5];
    t_range = [1,4];
    fprintf('The range for a is %f and %f \n', a_range(1), a_range(2));
    fprintf('The range for a_prime is %f and %f \n', a_prime_range(1), a_prime_range(2));
    fprintf('The range for h is %f and %f \n', h_range(1), h_range(2));
    fprintf('The range for t is %f and %f \n \n', t_range(1), t_range(2));

    %On file
    fprintf(fileID,'The range for a is %f and %f \n', a_range(1), a_range(2));
    fprintf(fileID, 'The range for a_prime is %f and %f \n', a_prime_range(1), a_prime_range(2));
    fprintf(fileID, 'The range for h is %f and %f \n', h_range(1), h_range(2));
    fprintf(fileID, 'The range for t is %f and %f \n \n', t_range(1), t_range(2));

    %50 valid points
    valid_points = 5*starts; %Number of starts = valid_points/5
    optimum = 0;
    fprintf('Please wait: Calculating the set of valid points for a initial simplex point that is non-singular in the desired RDW \n')
    if sobol_var == 0
        sobol_set = rand_tuning(n,a_range,a_prime_range,h_range,t_range,valid_points);
    else
         sobol_set = sobol_tuning(n,a_range,a_prime_range,h_range,t_range,valid_points);
    end
   
    fprintf('Done: Calculated initial simplexes for %d starts \n', valid_points/5);

    %Simplex S has 5 points each of dimension 4
    S_min = zeros(1,n);
    Best_simplex = zeros(n+1,n);
    multi_start_evaluations = zeros(1,valid_points/5);
    saved_S_eval = zeros(valid_points/5, n+1);
    %fprintf('The number of starts are %d \n', valid_points/5);
    fprintf('Number of iterations to stop if same solution is encountered are %d \n', steps);

    for multi_start = 1:starts

        S = sobol_set((5*(multi_start-1) +1):5*multi_start,:);
        %Now the initial simplex of valid points has been chosen

        %% Stage 2: Evaluating the objective function and initializing the simplex
        iteration = 1;
        prev_min_eval = 0;

        while(iteration <= steps) % stops if the solution has not changed for 10 iterations
            fprintf("The spherical joint limit is %f degrees \n", sphere_limit);
            evaluations = zeros(n+1,1);
            fprintf('\nPlease wait: Running the %d start \n', multi_start);
            for i = 1:5
                [evaluations(i),~] = obj_func_act(S(i,1), S(i,2), S(i,3), S(i,4),ball_halfwidth);
                %It is similar to evaluate = obj_funct(a,a_prime,h,t)
            end

            [max_eval,max_index] = max(evaluations); %To calculate the point correspoding to the worst evaluation
            [min_eval,min_index] = min(evaluations); %To calculate the point correspoding to the worst evaluation
            %We also need to save the second max value
            evaluations_backup = evaluations;
            evaluations_backup(max_index) = -inf;
            [second_max_eval, ~] = max(evaluations_backup);

            %% Stopping conditions
            % If the optimised value has been reached
            %i.e The simplex has shrinked below an acceptable epsilon1 value and the
            %function evaluation of all the points of simplex are identical with a
            %tolerance of epsilon2 then it is good time to stop
            epsilon1 = 0.001;
            epsilon2 = 0.001;
            stop_j = 2;
            stop_k =1;
            simplex_length = zeros(1,0.5*(n^2 +n));%0.5*(n^2 +n) -> (n+1)^C_2
            eval_length = zeros(1,0.5*(n^2 +n));
            for i = stop_j:n+1
                simplex_length(stop_k) = norm((S(stop_j-1,:) - S(i,:)),2);
                eval_length(stop_k) = abs(evaluations(stop_j-1) - evaluations(i));
                stop_k = stop_k+1;
            end

            simplex_size = max(simplex_length);
            eval_size = max(eval_length);

            if simplex_size < epsilon1 && eval_size < epsilon2
                optimum = 1;
                break;
            end

            %Sorting is done for ease of understanding and operation
            Sort_S = zeros(n+1,n);
            j = 2;
            for i = 1:n+1
                if i == max_index
                    continue;
                elseif i == min_index
                    continue;
                else
                    Sort_S(j,:) = S(i,:);
                    j = j+1;
                end
            end

            Sort_S(1,:) = S(min_index,:); %So the Simplex is sorted such that the first element is min eval
            Sort_S(j,:) = S(max_index,:); %So the Simplex is sorted such that the last element is max eval
            S_min = Sort_S(1,:);

            %Calculating the new point (average of the first n sorted-points)
            mean_point = zeros(1,n);
            for i = 1:n
                mean_point(i) = sum(Sort_S(1:n,i))/n;
            end

            %% Stage 3: Successive reflection, Expansion, acontraction and Shrinkage
            %Setting co-efficients for the simplex
            r_coeff = 1; %Reflection co-efficient
            e_coeff = 2; %Expansion co-efficient
            k_coeff = 0.5; %Contraction co-efficient
            s_coeff = 0.5; %Shrinkage co-efficient

            %Reflection : reflect_point = mean_point + reflect_coeff*(mean_point - S(n+1,:))
            reflect_point = mean_point + r_coeff*(mean_point - Sort_S(n+1,:));
            reflect_point = range_respect(reflect_point,a_range,a_prime_range,h_range,t_range,n);
            [reflect_evaluation,~] = obj_func_act(reflect_point(1), reflect_point(2), reflect_point(3), reflect_point(4),ball_halfwidth);

            if min_eval < reflect_evaluation && reflect_evaluation < max_eval
                Sort_S(n+1,:) = reflect_point;

            elseif reflect_evaluation < min_eval
                min_eval = reflect_evaluation;
                S_min = reflect_point;
                expand_point = mean_point + e_coeff*(reflect_point-mean_point);
                expand_point = range_respect(expand_point,a_range,a_prime_range,h_range,t_range,n);
                [expand_evaluation, ~] = obj_func_act(expand_point(1), expand_point(2), expand_point(3), expand_point(4),ball_halfwidth);

                    if expand_evaluation < reflect_evaluation
                        min_eval = expand_evaluation;
                        S_min = expand_point;
                        Sort_S(n+1,:) = expand_point;
                        fprintf('Expansion in process \n');
                    else
                        Sort_S(n+1,:) = reflect_point;
                        fprintf('Reflection in process \n');
                    end

            elseif reflect_evaluation > second_max_eval

                if second_max_eval < reflect_evaluation && reflect_evaluation < max_eval
                    outside_contract_point = mean_point + k_coeff*(reflect_point-mean_point);
                    outside_contract_point = range_respect(outside_contract_point,a_range,a_prime_range,h_range,t_range,n);
                    [outside_contract_evaluation,~] = obj_func_act(outside_contract_point(1), outside_contract_point(2), outside_contract_point(3), outside_contract_point(4),ball_halfwidth);

                    if outside_contract_evaluation < reflect_evaluation
                        Sort_S(n+1,:) = outside_contract_point;
                        fprintf('Outside contraction in process \n');
                    else %Perform Shrinkage
                        fprintf('Outside contraction of no use: Shrinkage in process \n');
                        for i = 2:n+1
                          Sort_S(i,:) = Sort_S(i,:) - s_coeff*(Sort_S(i,:) - Sort_S(1,:));
                          Sort_S(i,:) = range_respect(Sort_S(i,:),a_range,a_prime_range,h_range,t_range,n);
                        end
                    end

                elseif reflect_evaluation > max_eval
                    inside_contract_point = mean_point - k_coeff*(mean_point-Sort_S(n+1,:));
                    inside_contract_point = range_respect(inside_contract_point,a_range,a_prime_range,h_range,t_range,n);
                    [inside_contract_evaluation,~] = obj_func_act(inside_contract_point(1), inside_contract_point(2), inside_contract_point(3), inside_contract_point(4),ball_halfwidth);

                    if inside_contract_evaluation < max_eval
                        Sort_S(n+1,:) = inside_contract_point;
                        fprintf('Inside contraction in process \n');
                    else %Perform Shrinkage
                        fprintf('Inside contraction of no use: Shrinkage in process \n');
                        for i = 2:n+1
                          Sort_S(i,:) = Sort_S(i,:) - s_coeff*(Sort_S(i,:) - Sort_S(1,:));
                          Sort_S(i,:) = range_respect(Sort_S(i,:),a_range,a_prime_range,h_range,t_range,n);
                        end
                    end
                end

            end

            S = Sort_S; % In order to be able to run in loop

            fprintf("Finished %dth loop \n", iteration);
            fprintf("Best evaluation yet is %d \n", min_eval);

            if min_eval < prev_min_eval
                iteration = 1;
            else
                iteration = iteration+1;
            end
            prev_min_eval = min_eval;
        end % This ends the while loop of one Nelder Mead single start iteration
        
         [~,rho_vec]= obj_func_act(S_min(1), S_min(2), S_min(3), S_min(4),ball_halfwidth);
         rho1_min = rho_vec(1);
         rho1_max = rho_vec(2);
         rho2_min = rho_vec(3);
         rho2_max = rho_vec(4);
            
        if optimum == 1
            fprintf("The optimised point is found \n");
            fprintf("a = %f, a_prime = %f, h = %f and t = %f \n", S_min(1), S_min(2), S_min(3), S_min(4));
            fprintf("The evaluation at this point is %d \n \n", min_eval);
            fprintf("The actuator ranges are[%f, %f, %f, %f] \n \n", rho1_min,rho1_max,rho2_min,rho2_max);
            
            fprintf(fileID, "\n\nThe optimised point is found \n");
            fprintf(fileID, "a = %f, a_prime = %f, h = %f and t = %f \n", S_min(1), S_min(2), S_min(3), S_min(4));
            fprintf(fileID, "The evaluation at this point is %d \n \n", min_eval);
            fprintf(fileID,"The actuator ranges are[%f, %f, %f, %f] \n \n", rho1_min,rho1_max,rho2_min,rho2_max);
        else
            fprintf("The simplex stopped after %d iterations \n", steps);
            fprintf("a = %f, a_prime = %f, h = %f and t = %f \n", S_min(1), S_min(2), S_min(3), S_min(4));
            fprintf("The evaluation at this point is %d \n", min_eval);
            fprintf("The actuator ranges are[%f, %f, %f, %f] \n \n", rho1_min,rho1_max,rho2_min,rho2_max);
            
            fprintf(fileID, "\n\nThe simplex stopped after %d iterations \n");
            fprintf(fileID, "a = %f, a_prime = %f, h = %f and t = %f \n", S_min(1), S_min(2), S_min(3), S_min(4));
            fprintf(fileID, "The evaluation at this point is %d \n \n ", min_eval);
            fprintf(fileID,"The actuator ranges are[%f, %f, %f, %f] \n \n", rho1_min,rho1_max,rho2_min,rho2_max);
        end

        %% End of Simplex of a single start - further processing

        saved_S_eval(multi_start,:) = [S_min,min_eval];
        if min_eval <= min(multi_start_evaluations)
            Best_point_yet = S_min;
            Best_simplex = S;
            Best_rho = rho_vec;
            fprintf("Best point yet is [%f,%f,%f,%f] after %d iterations \n", Best_point_yet(1),Best_point_yet(2),Best_point_yet(3),Best_point_yet(4), multi_start);
            fprintf(fileID, "Best point yet is [%f,%f,%f,%f] after %d iterations \n", S_min(1),S_min(2),S_min(3),S_min(4), multi_start);
        else
            fprintf("Best point yet is [%f,%f,%f,%f] after %d iterations \n", Best_point_yet(1),Best_point_yet(2),Best_point_yet(3),Best_point_yet(4), multi_start);
            fprintf(fileID, "Best point yet is [%f,%f,%f,%f] after %d iterations \n", S_min(1),S_min(2),S_min(3),S_min(4), multi_start);
        end

        multi_start_evaluations(multi_start) = min_eval;


    end

    fprintf('Completed all starts of Nelder Mead: Closing file 1 \n');
    fclose(fileID);

    fprintf('Opening a file named Points_record.txt to save all the optimised points with different starting simplex \n \n');
    fileID2 = fopen(points_file_name,'a');
    fprintf(fileID2,'Report: \n The spherical limit is %f degrees \n The actuator constrained are considered', sphere_limit);
    fprintf(fileID2,'\n The number of starts used are %d', multi_start);
    fprintf(fileID2,'\n The algorithm stops when the simplex has shrunk to a size of %f or when encounters the same solution for %d times \n \n', epsilon1, steps);
    for i = 1:size(saved_S_eval,1)
      fprintf(fileID2, "[%f,%f,%f,%f],{%d} \n \n", saved_S_eval(i,1),saved_S_eval(i,2),saved_S_eval(i,3),saved_S_eval(i,4), saved_S_eval(i,5));
    end
    fclose(fileID2);
    
    fileID3 = fopen('temp.txt','w');
    for i = 1:size(saved_S_eval,1)
      fprintf(fileID3, "%f %f %f %f\n", saved_S_eval(i,1),saved_S_eval(i,2),saved_S_eval(i,3),saved_S_eval(i,4));
    end
    fclose(fileID3);
end
