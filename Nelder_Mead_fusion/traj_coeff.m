function para_c = traj_coeff(t_initial, t_final, b_vec)

a_mat = [1, t_initial, t_initial^2, t_initial^3, t_initial^4, t_initial^5;
    0, 1, 2*t_initial, 3*t_initial^2, 4*t_initial^3, 5*t_initial^4;
    0, 0, 2, 6*t_initial, 12*t_initial^2, 20*t_initial^3;
    1, t_final, t_final^2, t_final^3, t_final^4, t_final^5;
    0, 1, 2*t_final, 3*t_final^2, 4*t_final^3, 5*t_final^4;
    0, 0, 2, 6*t_final, 12*t_final^2, 20*t_final^3];
para_c = a_mat\b_vec;

end