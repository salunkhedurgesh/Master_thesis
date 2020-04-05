%%Nelder Mead implementation
%Stage 1: Choosing the simplex: Implementing sobol function for choosing
%         valid points with minimum discrepancy
%Stage 2: Stopping Criteria
%Stage 3: Evaluating the objective function and initializing the simplex
%Stage 4: Successive reflection, Expansion, contraction and Shrinkage
%Stage 5: Verifying the validity of the generated new point
%Code by: Durgesh Salunkhe, at Ecole Centrale de Nantes

function [S_min, rho_vec] = Nelder_mead_refine(S_raw)

    fprintf('Opening file named refine_report.text for storing the optimised points after each start \n');
    fileID = fopen('refine_report_already.txt','a');
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

    steps = 10;
    optimum = 0;
    %Simplex S has 5 points each of dimension 4
    S_min = zeros(1,n);

    fprintf('Number of iterations to stop if same solution is encountered are %d \n', steps);
    fprintf('Important: Now we are "NOT" using the parameter weightage \n');
    S = zeros(n+1,n);
        S(1,:) = S_raw;
        a_stroke = a_range(2)-a_range(1);
        a_prime_stroke = a_prime_range(2)-a_prime_range(1);
        h_stroke = h_range(2)-h_range(1);
        t_stroke = t_range(2)-t_range(1);
        percent_chart = [-1, 1, -1, -1; 1, 3, 1, -2; -2, 0, -3, -5; 1,-1,2,2];
        %percent_chart is the parameter to initialise the simplex around
        %the known solution. 1 -> 1% increment -1 -> 1% decrement etc...
        %percentage of what? ->they are individual percentages. 
        %so, 1% percent increment in the length of
        %a means 100% is the total range of a. 
        for i = 1:4
            S(i+1,:) = [S_raw(1)+(a_stroke*percent_chart(i,1))*0.01, S_raw(2)+(a_prime_stroke*percent_chart(i,2))*0.01, S_raw(3)+(h_stroke*percent_chart(i,3))*0.01, S_raw(4)+(t_stroke*percent_chart(i,4))*0.01];
        end
        fprintf('The points are:\n');
        fprintf('%f %f %f %f\n', S(2,1),S(2,2),S(2,3),S(2,4));
        fprintf('%f %f %f %f\n', S(3,1),S(3,2),S(3,3),S(3,4));
        fprintf('%f %f %f %f\n', S(4,1),S(4,2),S(4,3),S(4,4));
        fprintf('%f %f %f %f\n', S(5,1),S(5,2),S(5,3),S(5,4));

        %% Stage 2: Evaluating the objective function and initializing the simplex
        iteration = 1;
        prev_min_eval = 0;

        while(iteration <= steps) % stops if the solution has not changed for 10 iterations
            fprintf("The spherical joint limit is %f degrees \n", sphere_limit);
            evaluations = zeros(n+1,1);
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
            epsilon1 = 0.0001;
            epsilon2 = 0.0001;
            fprintf('The epsilon value to stop is %f\n', epsilon1);
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

            if simplex_size < epsilon1 || eval_size < epsilon2
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
                mean_point(i) = sum(Sort_S(i,1:n))/n;
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

    fprintf('Completed all starts of Nelder Mead: Closing file refine_report.txt \n');
    fclose(fileID);

end
