function plot_limit(a, a_prime, h, t,act_trigger,multi_start)
    sphere_limit = 32.5;
    iter = 1;
    valid_iter = 1;
    ball_halfwidth = sind(sphere_limit);
    hk =0; %For simplicity in the beginning, I am not involving hk
    act_range = [t+h-0.5, t+h+0.5]; %Actuator stroke length, "t+h" is assumed to be the mid-position of the actuator
    
    for alpha = -1 : 0.01 :1
        for beta = -1 :0.01 :1
            [rho1, rho2, p_lim_uni, p_lim_sph] = configuration_space(a,a_prime,h,t,ball_halfwidth,alpha,beta);
            %Actuator lengths
            if act_trigger == 1
                if rho1 < act_range(1) || rho1 > act_range(2)
                    %If any instance(alpha,beta) does not satisfy actuator length,
                    %then skip that iteration and evaluate the quality as '0'
                    continue; 
                elseif rho2 < act_range(1) || rho2 > act_range(2)
                    %If any instance(alpha,beta) does not satisfy actuator length,
                    %then skip that iteration and evaluate the quality as '0'
                    continue;
                end
            else
                rho_vec(iter,1:2) = [rho1,rho2];
                iter = iter + 1;
            end
            %Passive limit - Universal joints
            if p_lim_uni == 0
                %If any instance(alpha,beta) does not satisfy passive limits of
                %the universal joint then skip that iteration and evaluate the quality as '0'
                continue;
            end
            
            %Passive limit - Spherical joints
            if p_lim_sph == 0
                %If any instance(alpha,beta) does not satisfy passive limits of 
                %the spherical joint then skip that iteration and evaluate the quality as '0'
                continue;
            end
            valid_points(valid_iter,1:2) = [alpha,beta];
            valid_iter = valid_iter + 1;
        end
    end
    figure()
    for loop_i = 1:length(valid_points)
        plot(valid_points(loop_i,1),valid_points(loop_i,2), '.b');
        hold on;
    end
    plot([-1,1,1,-1,-1],[-1,-1,1,1,-1], '-r', 'Linewidth', 2);
    axis([-3 3 -3 3]);
end
