function [rho1_range,rho2_range] = rho_range(a,a_prime,h,t)
    
    hk = 0;    
    s12_mobile = [a_prime;hk;h];
    s22_mobile = [hk;a_prime;h];
    r_i = 1;
    for alpha = -1:0.01:1
        for beta = -1:0.01:1
            fixed_T_mobile = trans_mat('z', t)*rot_mat('x', alpha)*rot_mat('y', beta);
            s12 = fixed_T_mobile*[s12_mobile;1];
            s22 = fixed_T_mobile*[s22_mobile;1];

            u11_T_fixed = inv(trans_mat('x',a));
            u21_T_fixed = inv(trans_mat('y',a));

            s12_u11 = u11_T_fixed*s12;
            s22_u21 = u21_T_fixed*s22;
            rho1_vec(r_i) = norm(s12_u11(1:3),2);
            rho2_vec(r_i) = norm(s22_u21(1:3),2);
            
%             rho1_vec(r_i) = sqrt(2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*cos(alpha)*a_prime*t-2*cos(beta)*a*a_prime-2*sin(beta)*a*h+a^2+a_prime^2+h^2+t^2);
%             rho2_vec(r_i) = sqrt(sin(beta)^2*h^2+(cos(alpha)*a_prime-sin(alpha)*cos(beta)*h-a)^2+(sin(alpha)*a_prime+cos(alpha)*cos(beta)*h+t)^2);
            r_i = r_i + 1;
        end
    end
    rho1_range = zeros(1,2);
    rho2_range = zeros(1,2);
    
    rho1_range(1) = min(rho1_vec);
    rho1_range(2) = max(rho1_vec);
    rho2_range(1) = min(rho2_vec);
    rho2_range(2) = max(rho2_vec);
end

