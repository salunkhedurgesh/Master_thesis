%%Nelder Mead implementation
%Stage 1: Choosing the simplex
%Stage 2: Evaluating the objective function and initializing the simplex
%Stage 3: Successive reflection, Expansion, acontraction and Shrinkage

close all;
clear all;
clc
%% Stage 1: Choosing the Simplex
%Dimension of a point in simplex = 4 (a, a_prime, h and t)
%We will use sobolset for generating points for multistarting the simplex
n = 4; %number of dimensions
p = sobolset(n,'Skip',1e3,'Leap',1e2);
p = scramble(p,'MatousekAffineOwen');

%First simplex of 5 points
a_range = [0.25,1.25];
a_prime_range = [0.25,2];
h_range = [-0.5,1.5];
t_range = [1,4];

%Simplex S has 5 points each of dimension 4
S = zeros(n+1,n);

%This loop is a while loop because the sobolset generates many points from 
%the given range of the parameters but we want those points that have no 
%singularity curves in the RDW of +- 1 radian square defined in the output 
%workspace parmetrized by alpha and beta
i = 1;
j = 1;
while(j<n+2) 
a(i) = a_range(1) + p(i,1)*(a_range(2) - a_range(1));
a_prime(i) = a_prime_range(1) + p(i,2)*(a_prime_range(2) - a_prime_range(1));
h(i) = h_range(1) + p(i,3)*(h_range(2) - h_range(1));
t(i) = t_range(1) + p(i,4)*(t_range(2) - t_range(1));
sing_bool = RDW_sing(a(i), a_prime(i), h(i), t(i));
    if sing_bool == 1
        S(j,:) = [a(i),a_prime(i),h(i),t(i)];
        j = j+1;
    end
i = i+1;
end

evaluations = zeros(n+1,1);
max_eval = 0;
for i = 1:5
    evaluations(i) = obj_func(S(i,1), S(i,2), S(i,3), S(i,4));  
    %It is similar to evaluate = obj_funct(a,a_prime,h,t)
end
[max_eval,max_index] = max(evaluations); %To calculate the point correspoding to the worst evaluation
[min_eval,min_index] = min(evaluations); %To calculate the point correspoding to the worst evaluation
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








