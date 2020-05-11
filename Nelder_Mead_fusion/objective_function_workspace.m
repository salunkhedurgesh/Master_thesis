% function name: objective_function_workspace()
% Description: The calculation of the evaluation of the feasible workspace
% Inputs: 
% 1. type of the mechanism
% 2. parameters (A (1xn) vector) 
% 3. limit for the joints
% Outpus: 
% 1. evaluation of the objective function for given set of parameters
% 2. optimised actuator range

function [c_qual,rho_vec] = objective_function_workspace(type, parameters, limits, reward)
    
    valid_iter = 1;
    [rho1_range,rho2_range] = rho_range(type, parameters);
    c_qual_only_joint = 0;
    rho_vec = zeros(1, 4);
    for alpha = -1 : 0.01 :1
        for beta = -1 :0.01 :1
            
            [det_val, p_lim_uni, p_lim_sph1, p_lim_sph2, rho_inst] = constraints(type, parameters, limits, alpha, beta, reward);
            %Singularity check
            if abs(det_val) < 0.005
                c_qual = inf;
                break; %If any instance(alpha,beta) is singular, then the parameter is not useful
            end   
           
            %Passive limit - Universal joints
            if p_lim_uni(1) == 0 || p_lim_uni(2) == 0
                %If any instance(alpha,beta) does not satisfy passive limits of
                %the universal joint then skip that iteration and evaluate the quality as '0'
                continue;
            end
            
            %Passive limit - Spherical joints
            if p_lim_sph1(1) == 0 || p_lim_sph1(2) == 0 || p_lim_sph2(1) == 0 || p_lim_sph2(2) == 0 
                %If any instance(alpha,beta) does not satisfy passive limits of 
                %the spherical joint then skip that iteration and evaluate the quality as '0'
                continue;
            end
            valid_points_vec(valid_iter,1:4) = [alpha, beta, rho_inst];
            valid_iter = valid_iter + 1;
            c_qual_only_joint = c_qual_only_joint + 1;
        end
    end
    
    stroke = limits(3); %This is the ratio of min and max actuator lengths and not the stroke length itself
    if rho1_range(2) > stroke*rho1_range(1) || rho2_range(2) > stroke*rho2_range(1) 
        %Maximizing the valid points by applying the actuator limits 
        j = 1;
        stroke_step = 3;
        xmin1_step = ((rho1_range(2)/stroke) - rho1_range(1))/stroke_step;
        xmin2_step = ((rho2_range(2)/stroke) - rho2_range(1))/stroke_step ;
        
        for x_min1 = rho1_range(1):xmin1_step:rho1_range(2)/stroke
            for x_min2 = rho2_range(1):xmin2_step:rho2_range(2)/stroke
                c_qual_temp = 0; %initializing the value to calculate cumulative quality
                for i = 1:length(valid_points_vec)
                    rho1 = valid_points_vec(i, 3);
                    rho2 = valid_points_vec(i, 4);
                    if rho1 < x_min1 || rho1 > stroke*x_min1
                        %If any instance(alpha,beta) does not satisfy actuator length,
                        %then skip that iteration and evaluate the quality as '0'
                        continue; 
                    elseif rho2 < x_min2 || rho2 > stroke*x_min2
                        %If any instance(alpha,beta) does not satisfy actuator length,
                        %then skip that iteration and evaluate the quality as '0'
                        continue;
                    end
                    c_qual_temp = c_qual_temp + 1;
                end
                c_qual_mat(j,1:3) = [c_qual_temp, x_min1,x_min2];
                j = j+1;
            end
        end
        [c_qual,act_index] = max(c_qual_mat(:,1));
        c_qual = -c_qual;
        rho_vec_min = [c_qual_mat(act_index,2),c_qual_mat(act_index,3)];
        rho_vec = [rho_vec_min(1), rho_vec_min(1)*stroke, rho_vec_min(2), rho_vec_min(2)*stroke];
    else
        c_qual = -c_qual_only_joint;
        rho_vec = [rho1_range,rho2_range];
    end
end
