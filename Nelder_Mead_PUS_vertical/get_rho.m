function [rho1,rho2] = get_rho(a,a_prime,h,t,offset,alpha,beta)
    
    hk = 0;    
    s12_mobile = [a_prime;hk;h];
    s22_mobile = [hk;a_prime;h];
    
    fixed_T_mobile = trans_mat('z', t)*rot_mat('x', alpha)*rot_mat('y', beta);
    s12 = fixed_T_mobile*[s12_mobile;1];
    s22 = fixed_T_mobile*[s22_mobile;1];

    rho1 = s12(3) + sqrt(a^2 - s12(2)^2 - (offset - s12(1))^2);
    rho2 = s22(3) + sqrt(a^2 - s22(1)^2 - (offset - s22(2))^2);
            
end

