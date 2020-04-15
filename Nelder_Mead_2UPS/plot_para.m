function [u12, u21, s12, s22, h_point, t_point, x_plus, x_minus, y_plus, y_minus] = plot_para(alpha, beta, a, a_prime, h, t)

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
    u12 = [a;0;0];
    u21 = [0,a,0];
    
    %The universal joint
    x_minus = fixed_T_mobile*[-0.2;0;0;1];
    x_plus = fixed_T_mobile*[0.2;0;0;1];
    y_minus = fixed_T_mobile*[0;-0.2;0;1];
    y_plus = fixed_T_mobile*[0;0.2;0;1];
    
end
    