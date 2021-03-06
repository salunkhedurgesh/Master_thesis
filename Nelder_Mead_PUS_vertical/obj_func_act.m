function [c_qual,rho_vec] = obj_func_act(a, a_prime, h, t, offset, ball_halfwidth)
    valid_iter = 1;
    hk =0; %For simplicity in the beginning, I am not involving hk
    [rho1_range,rho2_range] = rho_range(a,a_prime,h,t,offset);
    break_var = 0;
    c_qual_only_joint = 0;
    fprintf('temp_print in obj_func_act %f %f %f %f %f \n', a, a_prime, h, t, offset);
    for alpha = -1 : 0.01 :1
        for beta = -1 :0.01 :1
            
            %Singularity check
            jac_det = ((2*h^2*cos(beta)*sin(beta) - 2*h*sin(beta)*sin(alpha)*(offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha)))/(2*(- (offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 - h^2*sin(beta)^2 + a^2)^(1/2)) + h*sin(beta)*cos(alpha))*(h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha) + ((h*cos(beta)*cos(alpha) - a_prime*sin(beta)*cos(alpha))*(h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha)))/(- (a_prime*cos(beta) - offset + h*sin(beta))^2 - (h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha))^2 + a^2)^(1/2)) - (((a_prime*sin(alpha) + h*cos(beta)*cos(alpha))*(offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha)))/(- (offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 - h^2*sin(beta)^2 + a^2)^(1/2) - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*(a_prime*cos(beta)*cos(alpha) - (2*(a_prime*cos(beta)*sin(alpha) + h*sin(beta)*sin(alpha))*(h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha)) - 2*(h*cos(beta) - a_prime*sin(beta))*(a_prime*cos(beta) - offset + h*sin(beta)))/(2*(- (a_prime*cos(beta) - offset + h*sin(beta))^2 - (h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha))^2 + a^2)^(1/2)) + h*sin(beta)*cos(alpha));
            if jac_det < 0.005
                break_var = 1;
                break; %If any instance(alpha,beta) is singular, then the parameter is not useful
            end   
            
            [~, ~, p_lim_uni, p_lim_sph,imvar] = configuration_space_UR(a,a_prime,h,t,offset,ball_halfwidth,alpha,beta);
           
            if imvar == 1
                break_var = 1;
                fprintf('WARNING: IMAGINARY PISTON LIMITS, RECHECK RANGES \n');
                break
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
            valid_points_vec(valid_iter,1:2) = [alpha,beta];
            valid_iter = valid_iter + 1;
            c_qual_only_joint = c_qual_only_joint + 1;
        end
        if break_var == 1
            break
        end
    end
    
    if break_var == 1
        c_qual = inf;
        rho_vec = zeros(1,4);
    else
        stroke = 1.5; %This is the ratio of min and max actuator lengths and not the stroke length itself
        if rho1_range(2) > stroke*rho1_range(1) && rho2_range(2) > stroke*rho2_range(1) 
            %Maximizing the valid points by applying the actuator limits 
            j = 1;
            stroke_step = 3;
            xmin1_step = ((rho1_range(2)/stroke) - rho1_range(1))/stroke_step;
            xmin2_step = ((rho2_range(2)/stroke) - rho2_range(1))/stroke_step ;
            for x_min1 = rho1_range(1):xmin1_step:rho1_range(2)/stroke
                for x_min2 = rho2_range(1):xmin2_step:rho2_range(2)/stroke
                    c_qual_temp = 0; %initializing the value to calculate cumulative quality
                    for i = 1:length(valid_points_vec)
                        [rho1,rho2] = get_rho(a,a_prime,h,t,offset,valid_points_vec(i,1),valid_points_vec(i,2));
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
end
