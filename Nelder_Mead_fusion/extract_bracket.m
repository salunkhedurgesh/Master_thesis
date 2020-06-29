% function name: objective_function_workspace()
% Description: The calculation of the evaluation of the feasible workspace
% Inputs:
% 1. type of the mechanism
% 2. parameters (A (1xn) vector)
% 3. limit for the joints
% Outpus:
% 1. evaluation of the objective function for given set of parameters
% 2. optimised actuator range

function [] = extract_bracket(type, parameters, limits, reward)
%parameters = [1.305506 	0.175931 	0.015756 	0.931778 	-1.507154 	0.058928 	1.312649 	0.999608 	-0.059587 	0.329540 	1.420063 	0.019006 	3.986142];
valid_iter = 1;
[rho1_range,rho2_range] = rho_range(type, parameters);
c_qual_only_joint = 0;
rho_vec = zeros(1, 2);
temp_i = 1;
for alpha = -1 : 0.05 :1
    for beta = -1 :0.05 :1
        
        [~, ~, ~, ~, rho_inst, ~, ~] = constraints(type, parameters, limits, alpha, beta);
        %Singularity check
        rho_vecc(temp_i, :) = rho_inst;
        temp_i = temp_i + 1;
    end
    
end

m1 = min(rho_vecc(:,1));
m2 = min(rho_vecc(:,2));
p1 = [min(m1, m2), min(m1, m2)];
p2 = [min(m1, m2)*1.35, min(m1, m2)];
p3 = [min(m1, m2)*1.35, min(m1, m2)*1.35];
p4 = [min(m1, m2), min(m1, m2)*1.35];

fig1 = figure();
plot(rho_vecc(:,1), rho_vecc(:,2), '.b');
xlabel('actuator 1')
ylabel('actuator 2')
set(gca,'FontSize',20, 'FontName', 'CMU Serif')
n1 = min(rho_vecc(:,1)) + 0.2;
n2 = min(rho_vecc(:,2)) + 0.2;
o1 = min(rho_vecc(:,1)) + 0.4;
o2 = min(rho_vecc(:,2)) + 0.4;
q1 = [min(n1, n2), min(n1, n2)];
q2 = [min(n1, n2)*1.35, min(n1, n2)];
q3 = [min(n1, n2)*1.35, min(n1, n2)*1.35];
q4 = [min(n1, n2), min(n1, n2)*1.35];
r1 = [min(o1, o2), min(o1, o2)];
r2 = [min(o1, o2)*1.35, min(o1, o2)];
r3 = [min(o1, o2)*1.35, min(o1, o2)*1.35];
r4 = [min(o1, o2), min(o1, o2)*1.35];
hold on;
plot([q1(1), q2(1), q3(1), q4(1), q1(1)], [q1(2), q2(2), q3(2), q4(2), q1(2)], '--k', 'Linewidth', 1.5);
plot([p1(1), p2(1), p3(1), p4(1), p1(1)], [p1(2), p2(2), p3(2), p4(2), p1(2)], '--r', 'Linewidth', 1.5);
plot([r1(1), r2(1), r3(1), r4(1), r1(1)], [r1(2), r2(2), r3(2), r4(2), r1(2)], '--m', 'Linewidth', 1.5);
xlabel('actuator 1')
ylabel('actuator 2')
set(gca,'FontSize',20, 'FontName', 'CMU Serif')
hold on 

fig2 = figure();
s1 = 4 - (0.35*3.2)*0.5;
s2 = 4 - (0.35*3.5)*0.5;
s3 = 4 - (0.35*3.8)*0.5;
plot([s1, s1+0.35*3.2, s1+0.35*3.2, s1, s1], [s1, s1, s1+0.35*3.2, s1+0.35*3.2, s1], '-r', 'Linewidth', 1.5);
hold on
plot([s2, s2+0.35*3.5, s2+0.35*3.5, s2, s2], [s2, s2, s2+0.35*3.5, s2+0.35*3.5, s2], '-k', 'Linewidth', 1.5);
plot([s3, s3+0.35*3.8, s3+0.35*3.8, s3, s3], [s3, s3, s3+0.35*3.8, s3+0.35*3.8, s3], '-m', 'Linewidth', 1.5);
plot([3,5], [3,5], '.b');
xlabel('actuator 1')
ylabel('actuator 2')
set(gca,'FontSize',20, 'FontName', 'CMU Serif')
end