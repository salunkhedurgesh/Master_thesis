function [rho1,rho2] = get_rho(a,a_prime,h,t,alpha,beta)
    
    hk = 0;    
    s12_mobile = [a_prime;hk;h];
    s22_mobile = [hk;a_prime;h];
    
    fixed_T_mobile = trans_mat('z', t)*rot_mat('x', alpha)*rot_mat('y', beta);
    s12 = fixed_T_mobile*[s12_mobile;1];
    s22 = fixed_T_mobile*[s22_mobile;1];

    u11_T_fixed = inv(trans_mat('x',a));
    u21_T_fixed = inv(trans_mat('y',a));

    s12_u11 = u11_T_fixed*s12;
    s22_u21 = u21_T_fixed*s22;

    rho1 = norm(s12_u11(1:3),2);
    rho2 = norm(s22_u21(1:3),2);
            
end

