function passive_limit_universal = p_lim_uni(a,a_prime, h, t, ball_halfwidth, alpha, beta, tilt_axis_u11,tilt_angle)
    hk = 0;

    R_u11 = rodrig_mat3axis(tilt_axis_u11, tilt_angle);
    rot_z_vec = R_u11*[0;0;1];
    rot_x_vec = R_u11*[1;0;0];

    if (abs(rot_z_vec(2)) > ball_halfwidth)
        passive_limit_universal(1) = 0;
    elseif (abs(rot_x_vec(2)) > ball_halfwidth)
            passive_limit_universal(1) = 0;
    else
        passive_limit_universal(1) = 1;
    end

    R_u21 = rodrig_mat3axis(tilt_axis_u21, second_tilt_angle);
    rot_z_vec = R_u21*[0;0;1];
    rot_y_vec = R_u21*[0;1;0];

    if (abs(rot_z_vec(1)) > ball_halfwidth)
        passive_limit_universal(2) = 0;
    elseif (abs(rot_y_vec(1)) > ball_halfwidth)
            passive_limit_universal(2) = 0;
    else
        passive_limit_universal(2) = 1;
    end
end
