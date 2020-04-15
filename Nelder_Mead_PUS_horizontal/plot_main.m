close all
clc

ball_halfwidth = sind(45);
a_prompt = 'Value of a \n';
a_prime_prompt = 'Value of a_prime \n';
h_prompt = 'Value of h \n';
t_prompt = 'Value of t \n';

a = input(a_prompt);
a_prime = input(a_prime_prompt);
h = input(h_prompt);
t = input(t_prompt);
[c_qual,S_rho] = obj_func_act(a, a_prime, h, t, ball_halfwidth);
S = [a, a_prime, h, t];
plot_code(S);
plot_points(S, S_rho, ball_halfwidth);

fprintf('The percentage of feasible workspace is %f %% \n', -c_qual/404);