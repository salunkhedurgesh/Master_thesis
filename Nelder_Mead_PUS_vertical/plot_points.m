%h = 0; hk= 0; a = 1; a_prime = 1.25; t = 1;
function [] = plot_points(S_min, rho_vec, ball_halfwidth)
    a = S_min(1);
    a_prime = S_min(2);
    h = S_min(3);
    t = S_min(4);
    offset = S_min(5);
    
    valid_iter = 1;
    hk =0; %For simplicity in the beginning, I am not involving hk
    
    j1 = 1; j2 = 1; j3 = 1; j4 = 1; j5 = 1; j6 = 1; j7 = 1;
    jac_vec(1,:) = [-2.8,-2.8]; 
    uni_vec1(1,:) = [-2.8,-2.8]; uni_vec2(1,:) = [-2.8,-2.8]; 
    sph_vec1(1,:) = [-2.8,-2.8]; sph_vec2(1,:) = [-2.8,-2.8];
    rho_vec1(1,:) = [-2.8,-2.8]; rho_vec2(1,:) = [-2.8,-2.80];
    
    c_qual_only_joint = 0;
    for alpha = -3 : 0.1 :3
        for beta = -3 :0.1 :3
            
            [rho1,rho2] = get_rho(a,a_prime,h,t, offset,alpha, beta);
            
            %Singularity check
            val = cos(beta)^2*sin(alpha)^2*a*h*hk*t-cos(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+cos(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2+sin(beta)*sin(alpha)*cos(alpha)*a*a_prime*h*t-sin(beta)*sin(alpha)*cos(alpha)*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)^2*a*a_prime*hk*t+cos(beta)*sin(beta)*sin(alpha)*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)*a*h*hk*t-cos(beta)*sin(beta)*sin(alpha)^2*a*a_prime*hk*t+cos(beta)^2*cos(alpha)*a^2*h^2+cos(beta)*cos(alpha)^2*a_prime^2*t^2-cos(beta)*cos(alpha)^2*hk^2*t^2-sin(beta)*sin(alpha)*a^2*a_prime^2-sin(beta)*cos(alpha)*a*a_prime^2*t+sin(beta)*cos(alpha)^2*a_prime*h*t^2-sin(beta)*cos(alpha)^2*h*hk*t^2+cos(beta)*sin(alpha)*a^2*a_prime*h-cos(beta)^2*sin(alpha)*a*h^2*t+sin(beta)^2*cos(alpha)*a^2*a_prime*hk+cos(beta)^2*cos(alpha)^2*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)^2*a*h^2*t+cos(beta)*sin(beta)*cos(alpha)^2*a*h^2*t-sin(beta)^2*sin(alpha)^2*a*a_prime*h*t-sin(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+sin(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2-sin(beta)^2*cos(alpha)^2*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)*a^2*a_prime*h-cos(beta)*sin(beta)*cos(alpha)*a^2*h*hk+cos(beta)*sin(alpha)*cos(alpha)*a*a_prime^2*t-cos(beta)*sin(alpha)*cos(alpha)*a*hk^2*t-sin(beta)^2*sin(alpha)*a*a_prime*hk*t+cos(beta)*cos(alpha)*a*a_prime*h*t;
            val_den = sqrt(2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*cos(alpha)*a_prime*t-2*cos(beta)*a*a_prime-2*sin(beta)*a*h+2*sin(alpha)*hk*t+a^2+a_prime^2+h^2+hk^2+t^2)*sqrt(2*cos(beta)*sin(alpha)*a*h+2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*sin(alpha)*a*hk-2*sin(beta)*cos(alpha)*hk*t+2*sin(alpha)*a_prime*t-2*cos(alpha)*a*a_prime+a^2+a_prime^2+h^2+hk^2+t^2);
            jac_det = abs(val/val_den);
            if jac_det < 0.005
                jac_vec(j1,1:2) = [alpha,beta];
                j1 = j1 + 1;
            end   
            
            [p_lim_uni1, p_lim_uni2, p_lim_sph1, p_lim_sph2] = configuration_space_PP(a,a_prime,h,t,offset,ball_halfwidth,alpha,beta);
           
            %Passive limit - Universal joints
            if p_lim_uni1 == 0
                %If any instance(alpha,beta) does not satisfy passive limits of
                %the universal joint then skip that iteration and evaluate the quality as '0'
                uni_vec1(j2,1:2) = [alpha,beta];
                j2 = j2 + 1;
            end
            
            if p_lim_uni2 == 0
                %If any instance(alpha,beta) does not satisfy passive limits of
                %the universal joint then skip that iteration and evaluate the quality as '0'
                uni_vec2(j3,1:2) = [alpha,beta];
                j3 = j3 + 1;
            end
            
            %Passive limit - Spherical joints
            if p_lim_sph1 == 0
                %If any instance(alpha,beta) does not satisfy passive limits of 
                %the spherical joint then skip that iteration and evaluate the quality as '0'
                sph_vec1(j4,1:2) = [alpha,beta];
                j4 = j4 + 1;
            end
            
            if p_lim_sph2 == 0
                %If any instance(alpha,beta) does not satisfy passive limits of 
                %the spherical joint then skip that iteration and evaluate the quality as '0'
                sph_vec2(j5,1:2) = [alpha,beta];
                j5 = j5 + 1;
            end
            
            if rho1 < rho_vec(1) || rho1 > rho_vec(2)
                        %If any instance(alpha,beta) does not satisfy actuator length,
                        %then skip that iteration and evaluate the quality as '0'
                        rho_vec1(j6,1:2) = [alpha,beta];
                        j6 = j6 + 1; 
                elseif rho2 < rho_vec(3) || rho2 > rho_vec(4)
                        %If any instance(alpha,beta) does not satisfy actuator length,
                        %then skip that iteration and evaluate the quality as '0'
                        rho_vec2(j7,1:2) = [alpha,beta];
                        j7 = j7 + 1; 
            end
        end
    end
    
    figure()
    %val = cos(beta)^2*sin(alpha)^2*a*h*hk*t-cos(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+cos(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2+sin(beta)*sin(alpha)*cos(alpha)*a*a_prime*h*t-sin(beta)*sin(alpha)*cos(alpha)*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)^2*a*a_prime*hk*t+cos(beta)*sin(beta)*sin(alpha)*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)*a*h*hk*t-cos(beta)*sin(beta)*sin(alpha)^2*a*a_prime*hk*t+cos(beta)^2*cos(alpha)*a^2*h^2+cos(beta)*cos(alpha)^2*a_prime^2*t^2-cos(beta)*cos(alpha)^2*hk^2*t^2-sin(beta)*sin(alpha)*a^2*a_prime^2-sin(beta)*cos(alpha)*a*a_prime^2*t+sin(beta)*cos(alpha)^2*a_prime*h*t^2-sin(beta)*cos(alpha)^2*h*hk*t^2+cos(beta)*sin(alpha)*a^2*a_prime*h-cos(beta)^2*sin(alpha)*a*h^2*t+sin(beta)^2*cos(alpha)*a^2*a_prime*hk+cos(beta)^2*cos(alpha)^2*a*a_prime*h*t+cos(beta)*sin(beta)*sin(alpha)^2*a*h^2*t+cos(beta)*sin(beta)*cos(alpha)^2*a*h^2*t-sin(beta)^2*sin(alpha)^2*a*a_prime*h*t-sin(beta)^2*sin(alpha)*cos(alpha)*a_prime*h*t^2+sin(beta)^2*sin(alpha)*cos(alpha)*h*hk*t^2-sin(beta)^2*cos(alpha)^2*a*h*hk*t-cos(beta)*sin(beta)*cos(alpha)*a^2*a_prime*h-cos(beta)*sin(beta)*cos(alpha)*a^2*h*hk+cos(beta)*sin(alpha)*cos(alpha)*a*a_prime^2*t-cos(beta)*sin(alpha)*cos(alpha)*a*hk^2*t-sin(beta)^2*sin(alpha)*a*a_prime*hk*t+cos(beta)*cos(alpha)*a*a_prime*h*t;
    plot(uni_vec1(:,1),uni_vec1(:,2), 'oc');
    title('Plot for the feasible workspace(blank space)');
    xlabel('circle - Universal joints, cross - Spherical joint, points - Actuators','FontSize',12);
    hold on;
    plot(uni_vec2(:,1),uni_vec2(:,2), 'om');
    plot(sph_vec1(:,1),sph_vec1(:,2), 'xc');
    plot(sph_vec2(:,1),sph_vec2(:,2), 'xm');
    plot(rho_vec1(:,1),rho_vec1(:,2), '.r');
    plot(rho_vec2(:,1),rho_vec2(:,2), '.b');
    plot([-1,1,1,-1,-1],[1,1,-1,-1,1], 'k', 'LineWidth', 2);
    f1 = @(alpha,beta) ((2*h^2*cos(beta)*sin(beta) - 2*h*sin(beta)*sin(alpha)*(offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha)))/(2*(- (offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 - h^2*sin(beta)^2 + a^2)^(1/2)) + h*sin(beta)*cos(alpha))*(h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha) + ((h*cos(beta)*cos(alpha) - a_prime*sin(beta)*cos(alpha))*(h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha)))/(- (a_prime*cos(beta) - offset + h*sin(beta))^2 - (h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha))^2 + a^2)^(1/2)) - (((a_prime*sin(alpha) + h*cos(beta)*cos(alpha))*(offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha)))/(- (offset - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 - h^2*sin(beta)^2 + a^2)^(1/2) - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*(a_prime*cos(beta)*cos(alpha) - (2*(a_prime*cos(beta)*sin(alpha) + h*sin(beta)*sin(alpha))*(h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha)) - 2*(h*cos(beta) - a_prime*sin(beta))*(a_prime*cos(beta) - offset + h*sin(beta)))/(2*(- (a_prime*cos(beta) - offset + h*sin(beta))^2 - (h*cos(beta)*sin(alpha) - a_prime*sin(beta)*sin(alpha))^2 + a^2)^(1/2)) + h*sin(beta)*cos(alpha));
    fimplicit(f1, '-r', 'Linewidth', 2)
    hold off
    
end
