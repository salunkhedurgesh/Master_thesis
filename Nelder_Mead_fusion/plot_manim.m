% function name: objective_function_plot()
% Description: The calculation of the evaluation of the feasible workspace
% Inputs:
% 1. type of the mechanism
% 2. parameters (A (1xn) vector)
% 3. limit for the joints
% Outpus:
% 1. evaluation of the objective function for given set of parameters
% 2. optimised actuator range

type = "2UPS";
parameters = [1.424194 	-1.704771 	-0.079518 	1.355703 	-0.144525 	0.173362 	0.870690 	1.626155 	-0.070115 	0.594410 	1.352876 	-0.040175 	2.953253 ];
rho_range = [3.098179 	4.647269 	2.554382 	3.831573 ];
limits = [45, 45, 1.5];
valid_iter = 1;
c_qual_only_joint = 0;
rho_vec = zeros(1, 4);
uinvalid1 = 1;
uinvalid2 = 1;
sinvalid11 = 1;
sinvalid12 = 1;
sinvalid21 = 1;
sinvalid22 = 1;
rinvalid1 = 1;
rinvalid2 = 1;

p_uni1_invalid(1,:) = [-3, -3];
p_uni2_invalid(1,:) = [-3, -3];
p_sph11_invalid(1,:) = [-2, 2];
p_sph12_invalid(1,:) = [-2, 2];
p_sph21_invalid(1,:) = [-2, 2];
p_sph22_invalid(1,:) = [-2, 2];

rho1_invalid(1, :) = [-2, -2];
rho2_invalid(1, :) = [-2, -2];

rowv = 1;
colv = 1;
for beta = 3 : -0.1 : -3
    for alpha = -3 : 0.1 : 3

        [~, p_lim_uni, p_lim_sph1, p_lim_sph2, rho_inst, ~, cn] = constraints(type, parameters, limits, alpha, beta);
        cnum(rowv, colv) = cn;
        colv = colv+1;
        %Passive limit - Universal joints
        if p_lim_uni(1) == 0
            %If any instance(alpha,beta) does not satisfy passive limits of
            %the universal joint then skip that iteration and evaluate the quality as '0'
            p_uni1_invalid(uinvalid1, 1:2) = [alpha, beta];
            uinvalid1 = uinvalid1 + 1;
        end
        
        if p_lim_uni(2) == 0
            %If any instance(alpha,beta) does not satisfy passive limits of
            %the universal joint then skip that iteration and evaluate the quality as '0'
            p_uni2_invalid(uinvalid2, 1:2) = [alpha, beta];
            uinvalid2 = uinvalid2 + 1;
        end
        
        %Passive limit - Spherical joints
        if p_lim_sph1(1) == 0
            %If any instance(alpha,beta) does not satisfy passive limits of
            %the spherical joint then skip that iteration and evaluate the quality as '0'
            p_sph11_invalid(sinvalid11, 1:2) = [alpha, beta];
            sinvalid11 = sinvalid11 + 1;
        end
        
        if p_lim_sph1(2) == 0
            %If any instance(alpha,beta) does not satisfy passive limits of
            %the spherical joint then skip that iteration and evaluate the quality as '0'
            p_sph12_invalid(sinvalid12, 1:2) = [alpha, beta];
            sinvalid12 = sinvalid12 + 1;
        end
        
        %Passive limit - Spherical joints
        if p_lim_sph2(1) == 0
            %If any instance(alpha,beta) does not satisfy passive limits of
            %the spherical joint then skip that iteration and evaluate the quality as '0'
            p_sph21_invalid(sinvalid21, 1:2) = [alpha, beta];
            sinvalid21 = sinvalid21 + 1;
        end
        
        if p_lim_sph2(2) == 0
            %If any instance(alpha,beta) does not satisfy passive limits of
            %the spherical joint then skip that iteration and evaluate the quality as '0'
            p_sph22_invalid(sinvalid22, 1:2) = [alpha, beta];
            sinvalid22 = sinvalid22 + 1;
        end
        
        if rho_inst(1) < rho_range(1) || rho_inst(1) > rho_range(2)
            rho1_invalid(rinvalid1, 1:2) = [alpha, beta];
            rinvalid1 = rinvalid1 + 1;
        end
        
        if rho_inst(2) < rho_range(3) || rho_inst(2) > rho_range(4)
            rho2_invalid(rinvalid2, 1:2) = [alpha, beta];
            rinvalid2 = rinvalid2 + 1;
        end
    end
    rowv = rowv+1;
    colv = 1;
end

u11x = parameters(1) * cos(parameters(2));
u11y = parameters(1) * sin(parameters(2));
u11z = parameters(3);

s12x = parameters(4) * cos(parameters(5));
s12y = parameters(4) * sin(parameters(5));
s12z = parameters(6);

u21x = parameters(7) * cos(parameters(8));
u21y = parameters(7) * sin(parameters(8));
u21z = parameters(9);

s22x = parameters(10) * cos(parameters(11));
s22y = parameters(10) * sin(parameters(11));
s22z = parameters(12);

t = parameters(13);

figure()
    %val = cos(beta)^2*sin(alpha)^2*a*h*hk*t-cos(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+cos(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2+sin(beta)*sin(alpha)*cos(alpha)*a*a_prime*h*t-sin(beta)*sin(alpha)*cos(alpha)*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)^2*a*a_prime*hk*t+cos(beta)*sin(beta)*sin(alpha)*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)*a*h*hk*t-cos(beta)*sin(beta)*sin(alpha)^2*a*a_prime*hk*t+cos(beta)^2*cos(alpha)*a^2*h^2+cos(beta)*cos(alpha)^2*a_prime^2*t^2-cos(beta)*cos(alpha)^2*hk^2*t^2-sin(beta)*sin(alpha)*a^2*a_prime^2-sin(beta)*cos(alpha)*a*a_prime^2*t+sin(beta)*cos(alpha)^2*a_prime*h*t^2-sin(beta)*cos(alpha)^2*h*hk*t^2+cos(beta)*sin(alpha)*a^2*a_prime*h-cos(beta)^2*sin(alpha)*a*h^2*t+sin(beta)^2*cos(alpha)*a^2*a_prime*hk+cos(beta)^2*cos(alpha)^2*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)^2*a*h^2*t+cos(beta)*sin(beta)*cos(alpha)^2*a*h^2*t-sin(beta)^2*sin(alpha)^2*a*a_prime*h*t-sin(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+sin(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2-sin(beta)^2*cos(alpha)^2*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)*a^2*a_prime*h-cos(beta)*sin(beta)*cos(alpha)*a^2*h*hk+cos(beta)*sin(alpha)*cos(alpha)*a*a_prime^2*t-cos(beta)*sin(alpha)*cos(alpha)*a*hk^2*t-sin(beta)^2*sin(alpha)*a*a_prime*hk*t+cos(beta)*cos(alpha)*a*a_prime*h*t;
    plot(p_uni1_invalid(:,1),p_uni1_invalid(:,2), 'oc');
    title('Plot for the feasible workspace(blank space)');
    xlabel('circle - Universal joints, cross - Spherical joint, points - Actuators','FontSize',12);
    hold on;
    plot(p_uni2_invalid(:,1),p_uni2_invalid(:,2), 'om');
    plot(p_sph11_invalid(:,1),p_sph11_invalid(:,2), 'xc');
    plot(p_sph12_invalid(:,1),p_sph12_invalid(:,2), 'xm');
    plot(p_sph21_invalid(:,1),p_sph21_invalid(:,2), 'xc');
    plot(p_sph22_invalid(:,1),p_sph22_invalid(:,2), 'xm');
    plot(rho1_invalid(:,1),rho1_invalid(:,2), '.r');
    plot(rho2_invalid(:,1),rho2_invalid(:,2), '.b');
    plot([-1,1,1,-1,-1],[1,1,-1,-1,1], 'k', 'LineWidth', 2);
    hold off;
    axis([-3 3 -3 3]);
figure()
    h = hmap(parameters);
