function passive_limit_universal = p_lim_uni(a,a_prime, h, t, ball_halfwidth, alpha, beta)
    hk = 0;
    tilt_axis_u11 = zeros(3,1);
    tilt_axis_u11(1) = -sin(alpha)*sin(beta)*a_prime-cos(alpha)*hk+sin(alpha)*cos(beta)*h;
    tilt_axis_u11(2) = cos(beta)*a_prime+sin(beta)*h-a;
    tilt_axis_u11(1) = tilt_axis_u11(1)/norm(tilt_axis_u11,2);
    tilt_axis_u11(1) = tilt_axis_u11(2)/norm(tilt_axis_u11,2);

    tilt_angle = acos((-cos(alpha)*sin(beta)*a_prime+sin(alpha)*hk+cos(alpha)*cos(beta)*h+t)/sqrt(2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*cos(alpha)*a_prime*t-2*cos(beta)*a*a_prime-2*sin(beta)*a*h+2*sin(alpha)*hk*t+a^2+a_prime^2+h^2+hk^2+t^2));
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

    tilt_axis_u21 = zeros(3,1);
    tilt_axis_u21(1) = -sin(alpha)*sin(beta)*hk-cos(alpha)*a_prime+sin(alpha)*cos(beta)*h+a;
    tilt_axis_u21(2) = cos(beta)*hk+sin(beta)*h;
    tilt_axis_u21(1) = tilt_axis_u21(1)/norm(tilt_axis_u21,2);
    tilt_axis_u21(1) = tilt_axis_u21(2)/norm(tilt_axis_u21,2);

    second_tilt_angle = acos((-cos(alpha)*sin(beta)*hk+sin(alpha)*a_prime+cos(alpha)*cos(beta)*h+t)/sqrt(2*cos(beta)*sin(alpha)*a*h+2*cos(beta)*cos(alpha)*h*t-2*sin(beta)*sin(alpha)*a*hk-2*sin(beta)*cos(alpha)*hk*t+2*sin(alpha)*a_prime*t-2*cos(alpha)*a*a_prime+a^2+a_prime^2+h^2+hk^2+t^2));

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
