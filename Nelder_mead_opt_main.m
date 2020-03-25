%%Nelder Mead implementation
%Stage 1: Choosing the simplex: Implementing sobol function for choosing 
%         valid points with minimum discrepancy
%Stage 2: Stopping Criteria
%Stage 3: Evaluating the objective function and initializing the simplex
%Stage 4: Successive reflection, Expansion, contraction and Shrinkage
%Stage 5: Verifying the validity of the generated new point
%Code by: Durgesh Salunkhe, at Ecole Centrale de Nantes

close all;
clear all;
clc
fileID = fopen('records_24march_3.txt','a');
%% Stage 1: Choosing the Simplex
% % % %Dimension of a point in simplex = 4 (a, a_prime, h and t)
% % % %We will use sobolset for generating points for multi_starting the simplex
n = 4; %number of dimensions
% % % p = sobolset(n,'Skip',1e3,'Leap',1e2);
% % % p = scramble(p,'MatousekAffineOwen');
% % % 
%First simplex of 5 points
a_range = [0.25,1.25];
a_prime_range = [0.25,2];
h_range = [-0.5,1.5];
t_range = [1,4];
%5000 valid points 
valid_points = 5000;
sobol_set = sobol_tuning(n,a_range,a_prime_range,h_range,t_range,1000);
%Simplex S has 5 points each of dimension 4
S = zeros(n+1,n);

multi_start_i = 1; %Its a variable used for choosing the valid points to
                   %initialise the multi-start Simplex with valid points
S_min = zeros(1,n);
multi_start_evaluations(1) = 0;

for multi_start = 1:valid_points/5
%This loop is because the sobolset generates many points from 
%the given range of the parameters but we want those points that have no 
%singularity curves in the RDW of +- 1 radian square defined in the output 
%workspace parmetrized by alpha and beta

i = multi_start;
S = sobol_set((5*(multi_start-1) +1):5*multi_start,:);
%Now the initial simplex of valid points has been chosen

%% Stage 2: Evaluating the objective function and initializing the simplex

signal = 1;
iteration = 1;

iteration = 1;
steps = 10;
prev_min_eval = 0;
while(iteration <= steps) % stops if the solution has not changed for 10 iterations
evaluations = zeros(n+1,1);
for i = 1:5
    evaluations(i) = obj_func(S(i,1), S(i,2), S(i,3), S(i,4));  
    %It is similar to evaluate = obj_funct(a,a_prime,h,t)
end
[max_eval,max_index] = max(evaluations); %To calculate the point correspoding to the worst evaluation
[min_eval,min_index] = min(evaluations); %To calculate the point correspoding to the worst evaluation
%We also need to save the second max value
evaluations_backup = evaluations;
evaluations_backup(max_index) = -inf;
[second_max_eval, second_max_index] = max(evaluations_backup);
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
    signal = 0;
    fprintf("The optimised point is found \n");
    fprintf("a = %f, a_prime = %f, h = %f and t = %f \n", S_min(1), S_min(2), S_min(3), S_min(4));
    fprintf("The evaluation at this point is %d \n", min_eval);
    fprintf(fileID, "\n\nThe optimised point is found \n");
    fprintf(fileID, "a = %f, a_prime = %f, h = %f and t = %f \n", S_min(1), S_min(2), S_min(3), S_min(4));
    fprintf(fileID, "The evaluation at this point is %d \n", min_eval);
    break;
end

%% 
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
reflect_point = range_respect(reflect_point,n);
reflect_evaluation = obj_func(reflect_point(1), reflect_point(2), reflect_point(3), reflect_point(4));
if min_eval < reflect_evaluation && reflect_evaluation < max_eval
    Sort_S(n+1,:) = reflect_point;
elseif reflect_evaluation < min_eval
    min_eval = reflect_evaluation;
    S_min = reflect_point;
    expand_point = mean_point + e_coeff*(reflect_point-mean_point);
    expand_point = range_respect(expand_point,n);
    expand_evaluation = obj_func(expand_point(1), expand_point(2), expand_point(3), expand_point(4));
        if expand_evaluation < reflect_evaluation
            min_eval = expand_evaluation;
            S_min = expand_point;
            Sort_S(n+1,:) = expand_point;
        else
            Sort_S(n+1,:) = reflect_point;
        end
elseif reflect_evaluation > second_max_eval
    if second_max_eval < reflect_evaluation && reflect_evaluation < max_eval
        outside_contract_point = mean_point + k_coeff*(reflect_point-mean_point);
        outside_contract_point = range_respect(outside_contract_point,n);
        outside_contract_evaluation = obj_func(outside_contract_point(1), outside_contract_point(2), outside_contract_point(3), outside_contract_point(4));
        if outside_contract_evaluation < reflect_evaluation
            Sort_S(n+1,:) = outside_contract_point;
        else %Perform Shrinkage
            for i = 2:n+1
              Sort_S(i,:) = Sort_S(i,:) - s_coeff*(Sort_S(i,:) - Sort_S(1,:));
            end
        end 
    elseif reflect_evaluation > max_eval
        inside_contract_point = mean_point - k_coeff*(mean_point-Sort_S(n+1,:));
        inside_contract_point = range_respect(inside_contract_point,n);
        inside_contract_evaluation = obj_func(inside_contract_point(1), inside_contract_point(2), inside_contract_point(3), inside_contract_point(4));
        if inside_contract_evaluation < max_eval
            Sort_S(n+1,:) = inside_contract_point;
        else %Perform Shrinkage
            for i = 2:n+1
              Sort_S(i,:) = Sort_S(i,:) - s_coeff*(Sort_S(i,:) - Sort_S(1,:));
              Sort_S(i,:) = range_respect(Sort_S(i,:),n);
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
end %% This ends the while loop of one Nelder Mead single start iteration
fprintf("The simplex stopped after %d iterations \n", steps);
fprintf("a = %f, a_prime = %f, h = %f and t = %f \n", S_min(1), S_min(2), S_min(3), S_min(4));
fprintf("The evaluation at this point is %d \n", min_eval);

saved_S_eval(multi_start,:) = [S_min,min_eval];
if min_eval <= min(multi_start_evaluations)
Best_point_yet = S_min;
fprintf("Best point yet is [%f,%f,%f,%f] after %d iterations \n", S_min(1),S_min(2),S_min(3),S_min(4), multi_start);
fprintf(fileID, "Best point yet is [%f,%f,%f,%f] after %d iterations \n", S_min(1),S_min(2),S_min(3),S_min(4), multi_start);
end

multi_start_evaluations(multi_start) = min_eval;
fprintf("Best evaluation with multi_start yet is %d after %d iterations \n", min(multi_start_evaluations), multi_start);
fprintf(fileID, "Best evaluation with multi_start yet is %d after %d iterations \n", min(multi_start_evaluations), multi_start);

end
fclose(fileID);       

fileID2 = fopen('Pointsrecords_24march_3.txt','a');
for i = 1:length(saved_S_eval)
  fprintf(fileID, "[%f,%f,%f,%f],{%d} \n \n", saved_S_eval(i,1),saved_S_eval(i,2),saved_S_eval(i,3),saved_S_eval(i,4), saved_S_eval(i,5));
end
fclose(fileID2);


