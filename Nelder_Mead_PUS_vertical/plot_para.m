function [u12, u21, s12, s22, h_point, t_point, x_plus, x_minus, y_plus, y_minus, offset1, offset2] = plot_para(alpha, beta, a, a_prime, h, t, offset)

    hk = 0;
    %origin = [0;0;0];
    
    %u11 = [a;0;0];
    s12_mobile = [a_prime;hk;h];
    %u21 = [0;a;0];
    s22_mobile = [hk;a_prime;h];
    fixed_T_mobile = trans_mat('z', t)*rot_mat('x', alpha)*rot_mat('y', beta);
    h_point = fixed_T_mobile*[0;0;h;1];
    t_point = fixed_T_mobile*[0;0;0;1];
    s12 = fixed_T_mobile*[s12_mobile;1];
    s22 = fixed_T_mobile*[s22_mobile;1];
    
    rho1 = s12(3) + sqrt(a^2 - s12(2)^2 - (offset - s12(1))^2);
    rho2 = s22(3) + sqrt(a^2 - s22(1)^2 - (offset - s22(2))^2);
    
    offset1 = [offset;0;0];
    offset2 = [0;offset;0];
    u12_4d = trans_mat('x', offset)*trans_mat('z', rho1)*[0;0;0;1];
    u21_4d = trans_mat('y', offset)*trans_mat('z', rho2)*[0;0;0;1];
    
    u12 = u12_4d(1:3);
    u21 = u21_4d(1:3);
    
    %The universal joint
    x_minus = fixed_T_mobile*[-0.2;0;0;1];
    x_plus = fixed_T_mobile*[0.2;0;0;1];
    y_minus = fixed_T_mobile*[0;-0.2;0;1];
    y_plus = fixed_T_mobile*[0;0.2;0;1];
    
end
    