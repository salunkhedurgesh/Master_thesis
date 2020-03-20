%h = 0; hk= 0; a = 1; a_prime = 1.25; t = 1;
function c_qual = obj_func(a, a_prime, h, t)
    sphere_limit = pi/6;
    ball_halfwidth = sin(sphere_limit);
    hk =0; %For simplicity in the beginning, I am not involving hk
    i = 0; %iteration keeper
    act_range = [t+h-0.5, t+h+0.5]; %Actuator stroke length, "t+h" is assumed to be the mid-position of the actuator
    quality = 0; %initializing the value to calculate quality for each loop
    c_qual = 0; %initializing the value to calculate cumulative quality 
    for alpha = -1 : 0.01 :1
        for beta = -1 :0.01 :1
            %Singularity check
            val = cos(beta)^2*sin(alpha)^2*a*h*hk*t-cos(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+cos(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2+sin(beta)*sin(alpha)*cos(alpha)*a*a_prime*h*t-sin(beta)*sin(alpha)*cos(alpha)*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)^2*a*a_prime*hk*t+cos(beta)*sin(beta)*sin(alpha)*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)*a*h*hk*t-cos(beta)*sin(beta)*sin(alpha)^2*a*a_prime*hk*t+cos(beta)^2*cos(alpha)*a^2*h^2+cos(beta)*cos(alpha)^2*a_prime^2*t^2-cos(beta)*cos(alpha)^2*hk^2*t^2-sin(beta)*sin(alpha)*a^2*a_prime^2-sin(beta)*cos(alpha)*a*a_prime^2*t+sin(beta)*cos(alpha)^2*a_prime*h*t^2-sin(beta)*cos(alpha)^2*h*hk*t^2+cos(beta)*sin(alpha)*a^2*a_prime*h-cos(beta)^2*sin(alpha)*a*h^2*t+sin(beta)^2*cos(alpha)*a^2*a_prime*hk+cos(beta)^2*cos(alpha)^2*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)^2*a*h^2*t+cos(beta)*sin(beta)*cos(alpha)^2*a*h^2*t-sin(beta)^2*sin(alpha)^2*a*a_prime*h*t-sin(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+sin(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2-sin(beta)^2*cos(alpha)^2*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)*a^2*a_prime*h-cos(beta)*sin(beta)*cos(alpha)*a^2*h*hk+cos(beta)*sin(alpha)*cos(alpha)*a*a_prime^2*t-cos(beta)*sin(alpha)*cos(alpha)*a*hk^2*t-sin(beta)^2*sin(alpha)*a*a_prime*hk*t+cos(beta)*cos(alpha)*a*a_prime*h*t;
            val_den = sqrt(2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*cos(alpha)*a_prime*t-2*cos(beta)*a*a_prime-2*sin(beta)*a*h+2*sin(alpha)*hk*t+a^2+a_prime^2+h^2+hk^2+t^2)*sqrt(2*cos(beta)*sin(alpha)*a*h+2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*sin(alpha)*a*hk-2*sin(beta)*cos(alpha)*hk*t+2*sin(alpha)*a_prime*t-2*cos(alpha)*a*a_prime+a^2+a_prime^2+h^2+hk^2+t^2);
            jac_det = abs(val/val_den);
            if jac_det < 0.01
                c_qual = 0;
                break; %If any instance(alpha,beta) is singular, then the parameter is not useful
            else
                quality = 1;
            end
            
            %Actuator lengths
            rho1 = sqrt(2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*cos(alpha)*a_prime*t-2*cos(beta)*a*a_prime-2*sin(beta)*a*h+2*sin(alpha)*hk*t+a^2+a_prime^2+h^2+hk^2+t^2);
            rho2 = sqrt(2*cos(beta)*sin(alpha)*a*h+2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*sin(alpha)*a*hk-2*sin(beta)*cos(alpha)*hk*t+2*sin(alpha)*a_prime*t-2*cos(alpha)*a*a_prime+a^2+a_prime^2+h^2+hk^2+t^2);
            
            if rho1 < act_range(1) || rho1 > act_range(2) 
                quality = 0;
                continue; %If any instance(alpha,beta) does not satisfy actuator length, then skip that iteration and evaluate the quality as '0'
            else if rho2 < act_range(1) || rho2 > act_range(2)
                    quality = 0;
                    continue;%If any instance (alpha,beta) does not satisfy actuator length, then skip that iteration and evaluate the quality as '0'
                end
            else
                quality = 1;
            end
            
            %Passive limit - Universal joints
            pass_lim_check_uni = p_lim_uni(a,a_prime, h, t, ball_halfwidth);
            if pass_lim_check_uni(1)*pass_lim_check_uni(2) == 0
                quality = 0;%If any instance(alpha,beta) does not satisfy passive limits of the universal joint
                            %, then skip that iteration and evaluate the quality as '0'
                continue;
            else
                quality = 1;
            end
            
            c_qual = c_qual + quality;
        end
    end
end