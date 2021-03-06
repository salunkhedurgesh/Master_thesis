%h = 0; hk= 0; a = 1; a_prime = 1.25; t = 1;
function [c_qual,rho_vec] = obj_func(a, a_prime, h, t,act_trigger)
    sphere_limit = 32.5;
    iter = 1;
    valid_iter = 1;
    ball_halfwidth = sind(sphere_limit);
    hk =0; %For simplicity in the beginning, I am not involving hk
    act_range = [t+h-0.5, t+h+0.5]; %Actuator stroke length, "t+h" is assumed to be the mid-position of the actuator
    c_qual = 0; %initializing the value to calculate cumulative quality

    for alpha = -1 : 0.01 :1
        for beta = -1 :0.01 :1
            
            %Starting by assuming that the instance (alpha,beta) satisfies all constraints
            quality = 1; 
            %Singularity check
            val = cos(beta)^2*sin(alpha)^2*a*h*hk*t-cos(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+cos(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2+sin(beta)*sin(alpha)*cos(alpha)*a*a_prime*h*t-sin(beta)*sin(alpha)*cos(alpha)*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)^2*a*a_prime*hk*t+cos(beta)*sin(beta)*sin(alpha)*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)*a*h*hk*t-cos(beta)*sin(beta)*sin(alpha)^2*a*a_prime*hk*t+cos(beta)^2*cos(alpha)*a^2*h^2+cos(beta)*cos(alpha)^2*a_prime^2*t^2-cos(beta)*cos(alpha)^2*hk^2*t^2-sin(beta)*sin(alpha)*a^2*a_prime^2-sin(beta)*cos(alpha)*a*a_prime^2*t+sin(beta)*cos(alpha)^2*a_prime*h*t^2-sin(beta)*cos(alpha)^2*h*hk*t^2+cos(beta)*sin(alpha)*a^2*a_prime*h-cos(beta)^2*sin(alpha)*a*h^2*t+sin(beta)^2*cos(alpha)*a^2*a_prime*hk+cos(beta)^2*cos(alpha)^2*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)^2*a*h^2*t+cos(beta)*sin(beta)*cos(alpha)^2*a*h^2*t-sin(beta)^2*sin(alpha)^2*a*a_prime*h*t-sin(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+sin(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2-sin(beta)^2*cos(alpha)^2*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)*a^2*a_prime*h-cos(beta)*sin(beta)*cos(alpha)*a^2*h*hk+cos(beta)*sin(alpha)*cos(alpha)*a*a_prime^2*t-cos(beta)*sin(alpha)*cos(alpha)*a*hk^2*t-sin(beta)^2*sin(alpha)*a*a_prime*hk*t+cos(beta)*cos(alpha)*a*a_prime*h*t;
            val_den = sqrt(2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*cos(alpha)*a_prime*t-2*cos(beta)*a*a_prime-2*sin(beta)*a*h+2*sin(alpha)*hk*t+a^2+a_prime^2+h^2+hk^2+t^2)*sqrt(2*cos(beta)*sin(alpha)*a*h+2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*sin(alpha)*a*hk-2*sin(beta)*cos(alpha)*hk*t+2*sin(alpha)*a_prime*t-2*cos(alpha)*a*a_prime+a^2+a_prime^2+h^2+hk^2+t^2);
            jac_det = abs(val/val_den);
            if jac_det < 0.005
                c_qual = -1/jac_det;
                break; %If any instance(alpha,beta) is singular, then the parameter is not useful
            end   
            
            [rho1, rho2, p_lim_uni, p_lim_sph] = configuration_space(a,a_prime,h,t,ball_halfwidth,alpha,beta);
            %Actuator lengths
            if act_trigger == 1
                rho_vec = zeros(1,2);
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
            c_qual = c_qual + quality;
        end
    c_qual = -c_qual; %This is necessary because I need to model an optimisation problem to minimize the objective function
    end
end
