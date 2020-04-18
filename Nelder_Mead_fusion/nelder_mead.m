% function name: nelder_mead()
% Description: Implementation of single start Nelder Mead
% Inputs:
% 1. type of the mechanism
% 2. initial simplex
% 3. objective_choice
% 4. ranges of the parameters
% 5. limit for the joints
% Outpus:
% 1. optimised set of parameter (highly possible: local minima)
% 2. optimised actuator range

function [single_best_point, single_best_rho, single_eval, optimum] = nelder_mead(type, S, iterations, objective_choice, ranges, limits, co_eff_mat)

    iteration = 1;
    prev_min_eval = 0;
    n = length(S(1,:));
    optimum = 0;
    while(iteration <= iterations) % stops if the solution has not changed for specified iterations
        evaluations = zeros(n+1,1);
        for i = 1:n+1
            [evaluations(i), ~] = objective_function(type, objective_choice, S(i,:), limits);
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
        %% Sortation and calculation of the mean point
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
        r_coeff = co_eff_mat(1); %Reflection co-efficient
        e_coeff = co_eff_mat(2); %Expansion co-efficient
        k_coeff = co_eff_mat(3); %Contraction co-efficient
        s_coeff = co_eff_mat(4); %Shrinkage co-efficient
        
        %Reflection : reflect_point = mean_point + reflect_coeff*(mean_point - S(n+1,:))
        reflect_point = mean_point + r_coeff*(mean_point - Sort_S(n+1,:));
        reflect_point = range_respect(reflect_point,ranges);
        [reflect_evaluation,~] = objective_function(type, objective_choice, reflect_point, limits);
        
        if min_eval < reflect_evaluation && reflect_evaluation < max_eval
            Sort_S(n+1,:) = reflect_point;
            fprintf('Reflection in process \n');
            
        elseif reflect_evaluation < min_eval
            min_eval = reflect_evaluation;
            S_min = reflect_point;
            expand_point = mean_point + e_coeff*(reflect_point-mean_point);
            % In other words:
            % expand_point = mean_point + e_coeff*r_coeff(mean_point - Sort_S(n+1,:));
            % It is necessary that e_coeff > 1 so that 
            % e_coeff*r_coeff > r_coeff
            
            expand_point = range_respect(expand_point,ranges);
            [expand_evaluation, ~] = objective_function(type, objective_choice, expand_point, limits);
            
            if expand_evaluation < reflect_evaluation
                min_eval = expand_evaluation;
                S_min = expand_point;
                Sort_S(n+1,:) = expand_point;
                fprintf('Expansion in process \n');
            else
                Sort_S(n+1,:) = reflect_point;
                fprintf('Checked expansion but reflection in process \n');
            end
            
        elseif reflect_evaluation > second_max_eval
            
            if second_max_eval < reflect_evaluation && reflect_evaluation < max_eval
                outside_contract_point = mean_point + k_coeff*(reflect_point-mean_point);
                outside_contract_point = range_respect(outside_contract_point, ranges);
                [outside_contract_evaluation,~] = objective_function(type, objective_choice, outside_contract_point, limits);
                
                if outside_contract_evaluation < reflect_evaluation
                    Sort_S(n+1,:) = outside_contract_point;
                    fprintf('Outside contraction in process \n');
                else %Perform Shrinkage
                    fprintf('Outside contraction of no use: Shrinkage in process \n');
                    for i = 2:n+1
                        Sort_S(i,:) = Sort_S(i,:) - s_coeff*(Sort_S(i,:) - Sort_S(1,:));
                        Sort_S(i,:) = range_respect(Sort_S(i,:),ranges);
                    end
                end
                
            elseif reflect_evaluation > max_eval
                inside_contract_point = mean_point - k_coeff*(mean_point-Sort_S(n+1,:));
                inside_contract_point = range_respect(inside_contract_point, ranges);
                [inside_contract_evaluation, ~] = objective_function(type, objective_choice, inside_contract_point, limits);
                
                if inside_contract_evaluation < max_eval
                    Sort_S(n+1,:) = inside_contract_point;
                    fprintf('Inside contraction in process \n');
                else %Perform Shrinkage
                    fprintf('Inside contraction of no use: Shrinkage in process \n');
                    for i = 2:n+1
                        Sort_S(i,:) = Sort_S(i,:) - s_coeff*(Sort_S(i,:) - Sort_S(1,:));
                        Sort_S(i,:) = range_respect(Sort_S(i,:),ranges);
                    end
                end
            end
            
        end
        
        S = Sort_S; % In order to be able to run in loop
        
        if min_eval < prev_min_eval
            iteration = 1;
            fprintf("Better point found.. continuing\n");
            fprintf("New evaluation is %d \n\n", min_eval);
        else
            iteration = iteration+1;
            fprintf("Same solution encountered\n");
            fprintf("The evaluation is %d \n\n", min_eval);
        end
        
        prev_min_eval = min_eval;
    end
    
    [single_eval, single_best_rho] = objective_function(type, objective_choice, S_min, limits);
    single_best_point = S_min;
    
    
end