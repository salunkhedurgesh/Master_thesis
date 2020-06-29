% function name: objective_function_plot()
% Description: The calculation of the evaluation of the feasible workspace
% Inputs:
% 1. type of the mechanism
% 2. parameters (A (1xn) vector)
% 3. limit for the joints
% Outpus:
% 1. evaluation of the objective function for given set of parameters
% 2. optimised actuator range

function [] = extract_plot_valid(type, parameters, limits, rho_range)

rinvalid1 = 1;
rinvalid2 = 1;

rho1_invalid(1, :) = [-2, -2];
rho2_invalid(1, :) = [-2, -2];

rowv = 1;
colv = 1;
for beta = 1 : -0.01 : -1
    for alpha = -1 : 0.01 : 1

        [~, p_lim_uni, p_lim_sph1, p_lim_sph2, rho_inst, ~, cn] = constraints(type, parameters, limits, alpha, beta);
             
        if rho_inst(1) < rho_range(1) || rho_inst(1) > rho_range(2)
            rho1_invalid(rinvalid1, 1:2) = [alpha, beta];
            rinvalid1 = rinvalid1 + 1;
        end
        
        if rho_inst(2) < rho_range(1) || rho_inst(2) > rho_range(2)
            rho2_invalid(rinvalid2, 1:2) = [alpha, beta];
            rinvalid2 = rinvalid2 + 1;
        end
    end
end

figure()
    plot(rho1_invalid(:,1),rho1_invalid(:,2), '.r');
    xlabel('alpha')
    ylabel('beta')
    set(gca,'FontSize',20)
    hold on;
    plot(rho2_invalid(:,1),rho2_invalid(:,2), '.b');
    hold off;
    axis([-1 1 -1 1]);
end
