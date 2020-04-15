%h = 0; hk= 0; a = 1; a_prime = 1.25; t = 1;
function [c_qual,rho_vec] = obj_func_para(a, a_prime, h, t, ball_halfwidth)
    valid_iter = 1;
    hk =0; %For simplicity in the beginning, I am not involving hk
    [rho1_range,rho2_range] = rho_range(a,a_prime,h,t);
    %parameter weightage
    para_weight = ((a + abs(h) + t)*(a_prime)) + 10;
    c_qual_only_joint = 0;
    
    for alpha = -1 : 0.01 :1
        for beta = -1 :0.01 :1
            
            %Singularity check
            jac_det = t*(sin(alpha)^3*sin(beta)*cos(beta)^2*a_prime^2*h-sin(alpha)^3*cos(beta)^3*a_prime*h^2+sin(alpha)*sin(beta)*cos(beta)^2*cos(alpha)^2*a_prime^2*h-sin(alpha)*cos(beta)^3*cos(alpha)^2*a_prime*h^2-sin(alpha)^2*sin(beta)^2*cos(alpha)*a_prime^2*h-sin(alpha)^2*sin(beta)*cos(beta)*cos(alpha)*a_prime^3+sin(alpha)^2*sin(beta)*cos(beta)*cos(alpha)*a_prime*h^2+sin(alpha)^2*cos(beta)^2*cos(alpha)*a_prime^2*h-sin(beta)^2*cos(alpha)^3*a_prime^2*h-sin(beta)*cos(beta)*cos(alpha)^3*a_prime^3+sin(beta)*cos(beta)*cos(alpha)^3*a_prime*h^2+cos(beta)^2*cos(alpha)^3*a_prime^2*h-sin(alpha)*sin(beta)^2*cos(alpha)*a_prime*h*t-sin(alpha)*cos(beta)^2*cos(alpha)*a_prime*h*t+sin(alpha)*sin(beta)*cos(beta)*sqrt(-sin(alpha)^2*sin(beta)^2*a_prime^2+2*sin(alpha)^2*sin(beta)*cos(beta)*a_prime*h-sin(alpha)^2*cos(beta)^2*h^2-cos(alpha)^2*sin(beta)^2*a_prime^2+2*cos(alpha)^2*sin(beta)*cos(beta)*a_prime*h-cos(alpha)^2*cos(beta)^2*h^2+2*cos(alpha)*sin(beta)*a_prime*t-2*cos(alpha)*cos(beta)*h*t+a^2-t^2)*a_prime*h-sin(alpha)*sin(beta)*cos(beta)*sqrt(-sin(alpha)^2*cos(beta)^2*h^2-cos(alpha)^2*cos(beta)^2*h^2-sin(alpha)^2*a_prime^2-2*cos(alpha)*cos(beta)*h*t-cos(alpha)^2*a_prime^2-2*sin(alpha)*a_prime*t+a^2-t^2)*a_prime*h-sin(alpha)*cos(beta)^2*sqrt(-sin(alpha)^2*sin(beta)^2*a_prime^2+2*sin(alpha)^2*sin(beta)*cos(beta)*a_prime*h-sin(alpha)^2*cos(beta)^2*h^2-cos(alpha)^2*sin(beta)^2*a_prime^2+2*cos(alpha)^2*sin(beta)*cos(beta)*a_prime*h-cos(alpha)^2*cos(beta)^2*h^2+2*cos(alpha)*sin(beta)*a_prime*t-2*cos(alpha)*cos(beta)*h*t+a^2-t^2)*h^2+sin(alpha)*cos(beta)^2*sqrt(-sin(alpha)^2*cos(beta)^2*h^2-cos(alpha)^2*cos(beta)^2*h^2-sin(alpha)^2*a_prime^2-2*cos(alpha)*cos(beta)*h*t-cos(alpha)^2*a_prime^2-2*sin(alpha)*a_prime*t+a^2-t^2)*h^2+sin(beta)*cos(alpha)^2*a_prime*h*t+cos(beta)*cos(alpha)^2*a_prime^2*t-cos(alpha)*sin(beta)*sqrt(-sin(alpha)^2*sin(beta)^2*a_prime^2+2*sin(alpha)^2*sin(beta)*cos(beta)*a_prime*h-sin(alpha)^2*cos(beta)^2*h^2-cos(alpha)^2*sin(beta)^2*a_prime^2+2*cos(alpha)^2*sin(beta)*cos(beta)*a_prime*h-cos(alpha)^2*cos(beta)^2*h^2+2*cos(alpha)*sin(beta)*a_prime*t-2*cos(alpha)*cos(beta)*h*t+a^2-t^2)*a_prime^2+cos(alpha)*cos(beta)*sqrt(-sin(alpha)^2*sin(beta)^2*a_prime^2+2*sin(alpha)^2*sin(beta)*cos(beta)*a_prime*h-sin(alpha)^2*cos(beta)^2*h^2-cos(alpha)^2*sin(beta)^2*a_prime^2+2*cos(alpha)^2*sin(beta)*cos(beta)*a_prime*h-cos(alpha)^2*cos(beta)^2*h^2+2*cos(alpha)*sin(beta)*a_prime*t-2*cos(alpha)*cos(beta)*h*t+a^2-t^2)*a_prime*h)/(sqrt(-sin(alpha)^2*sin(beta)^2*a_prime^2+2*sin(alpha)^2*sin(beta)*cos(beta)*a_prime*h-sin(alpha)^2*cos(beta)^2*h^2-cos(alpha)^2*sin(beta)^2*a_prime^2+2*cos(alpha)^2*sin(beta)*cos(beta)*a_prime*h-cos(alpha)^2*cos(beta)^2*h^2+2*cos(alpha)*sin(beta)*a_prime*t-2*cos(alpha)*cos(beta)*h*t+a^2-t^2)*sqrt(-sin(alpha)^2*cos(beta)^2*h^2-cos(alpha)^2*cos(beta)^2*h^2-sin(alpha)^2*a_prime^2-2*cos(alpha)*cos(beta)*h*t-cos(alpha)^2*a_prime^2-2*sin(alpha)*a_prime*t+a^2-t^2));
            if jac_det < 0.005
                break; %If any instance(alpha,beta) is singular, then the parameter is not useful
            end   
            
            [~, ~, p_lim_uni, p_lim_sph] = configuration_space_UR(a,a_prime,h,t,ball_halfwidth,alpha,beta);
           
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
    end
    
    stroke = 1.5; %This is the ratio of min and max actuator lengths and not the stroke length itself
    if rho1_range(2) > stroke*rho1_range(1) && rho2_range(2) > stroke*rho2_range(1) 
        %Maximizing the valid points by applying the actuator limits 
        j = 1;
        stroke_step = 2;
        xmin1_step = ((rho1_range(2)/stroke) - rho1_range(1))/stroke_step;
        xmin2_step = ((rho2_range(2)/stroke) - rho2_range(1))/stroke_step ;
        for x_min1 = rho1_range(1):xmin1_step:rho1_range(2)/stroke
            for x_min2 = rho2_range(1):xmin2_step:rho2_range(2)/stroke
                c_qual_temp = 0; %initializing the value to calculate cumulative quality
                for i = 1:length(valid_points_vec)
                    [rho1,rho2] = get_rho(a,a_prime,h,t,valid_points_vec(i,1),valid_points_vec(i,2));
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
        c_qual = c_qual/para_weight;
        c_qual = -c_qual;
        rho_vec_min = [c_qual_mat(act_index,2),c_qual_mat(act_index,3)];
        rho_vec = [rho_vec_min(1), rho_vec_min(1)*stroke, rho_vec_min(2), rho_vec_min(2)*stroke];
    else
        c_qual_only_joint = c_qual_only_joint/para_weight;
        c_qual = -c_qual_only_joint;
        rho_vec = [rho1_range,rho2_range];
    end
end
