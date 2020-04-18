% function name: constraints_UPS()
% Description: The calculation of the constraints of the mechanism at a
% certain configuration :checking for the constraints for only Universal 
% and Spherical joint
% Inputs: 
% 1. parameters (A (1xn) vector) 
% 2. limit for the joints
% 3. current configuration: alpha, beta
% Outpus: 
% 1. Determinant of the jacobian for a given configuration
% 1. [1 x 2] boolean vector for passive universal joint of both legs
% 2. [1 x 2] boolean vector for passive spherical joint of both legs

function [det_var, p_lim_uni, p_lim_sph, rho_inst] = constraints_UPS(parameters, limits, alpha, beta)
    
    a = parameters(1);
    a_prime = parameters(2);
    h = parameters(3);
    t = parameters(4);
    p_lim_uni = [1, 1];
    p_lim_sph = [1, 1];
    rho_inst = [0, 0];
    
    determinant = ((2*abs(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))*sign(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))*(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta)) - 2*abs(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))*sign(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))*(h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta)))*(2*h*abs(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*sign(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*sin(alpha)*sin(beta) - 2*h*sign(h*sin(beta))*cos(beta)*abs(h*sin(beta)) + 2*h*abs(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))*sign(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))*cos(alpha)*sin(beta)))/(4*(abs(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))^2 + abs(a + a_prime*cos(beta) + h*sin(beta))^2 + abs(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))^2)^(1/2)*(abs(h*sin(beta))^2 + abs(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 + abs(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))^2)^(1/2)) + ((2*abs(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*sign(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))*(a_prime*sin(alpha) + h*cos(alpha)*cos(beta)) + 2*abs(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))*sign(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))*(a_prime*cos(alpha) - h*cos(beta)*sin(alpha)))*(2*abs(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))*sign(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))*(a_prime*cos(alpha)*cos(beta) + h*cos(alpha)*sin(beta)) - 2*abs(a + a_prime*cos(beta) + h*sin(beta))*sign(a + a_prime*cos(beta) + h*sin(beta))*(h*cos(beta) - a_prime*sin(beta)) + 2*abs(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))*sign(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))*(a_prime*cos(beta)*sin(alpha) + h*sin(alpha)*sin(beta))))/(4*(abs(t + h*cos(alpha)*cos(beta) - a_prime*cos(alpha)*sin(beta))^2 + abs(a + a_prime*cos(beta) + h*sin(beta))^2 + abs(h*cos(beta)*sin(alpha) - a_prime*sin(alpha)*sin(beta))^2)^(1/2)*(abs(h*sin(beta))^2 + abs(a - a_prime*cos(alpha) + h*cos(beta)*sin(alpha))^2 + abs(t + a_prime*sin(alpha) + h*cos(alpha)*cos(beta))^2)^(1/2));
    det_var = "NS"; % non-singular
    if abs(determinant) < 0.005
        det_var = "singular";
        return;
    end
    
    abs_z = [0;0;1];
    origin_mobile = [0;0;t;1];
    s12_mobile = [a_prime;0;h];
    s22_mobile = [0;a_prime;h];

    fixed_T_mobile = trans_mat('z', t)*rot_mat('x', alpha)*rot_mat('y', beta);
    s12 = fixed_T_mobile*[s12_mobile;1];
    s22 = fixed_T_mobile*[s22_mobile;1];

    u11_T_fixed = inv(trans_mat('x',a));
    u21_T_fixed = inv(trans_mat('y',a));

    s12_u11 = u11_T_fixed*s12;
    s22_u21 = u21_T_fixed*s22;
    
    rho1 = norm(s12_u11(1:3),2);
    rho2 = norm(s22_u21(1:3),2);
    rho_inst = [rho1, rho2];

    tilt_axis = cross(abs_z,s12_u11(1:3));
    second_tilt_axis = cross(abs_z,s22_u21(1:3));

    dot_vectoru11 = dot(abs_z,s12_u11(1:3))/norm(s12_u11(1:3),2);
    dot_vectoru21 = dot(abs_z,s22_u21(1:3))/norm(s22_u21(1:3),2);

    tilt_angle = acos(dot_vectoru11);
    second_tilt_angle = acos(dot_vectoru21);

    rot_mat1inv = eye(4);
    second_rot_mat1inv = eye(4);
    rot_mat1inv(1:3,1:3) = transpose(rodrig_mat3axis(tilt_axis,tilt_angle));
    second_rot_mat1inv(1:3,1:3) = transpose(rodrig_mat3axis(tilt_axis,tilt_angle));

    sub_vect = fixed_T_mobile*[0;0;h;1];
    second_sub_vect = fixed_T_mobile*[0;0;h;1];

    ee_x_s12 = [-a_prime;0;0];
    ee_y_s22 = [0;-a_prime;0];

    x_current = rot_mat1inv(1:3,1:3)*(sub_vect(1:3) - s12(1:3));
    second_y_current = second_rot_mat1inv(1:3,1:3)*(second_sub_vect(1:3) - s22(1:3));

    tilt_axis2 = cross(ee_x_s12, x_current);
    second_tilt_axis2 = cross(ee_y_s22, second_y_current);

    tilt_angle2 = acos((dot(ee_x_s12,x_current))/(norm(ee_x_s12,2)*norm(x_current,2)));
    second_tilt_angle2 = acos((dot(ee_y_s22,second_y_current))/(norm(ee_y_s22,2)*norm(second_y_current,2)));

    rot_mat2inv = eye(4);
    second_rot_mat2inv = eye(4);
    rot_mat2inv(1:3,1:3) = transpose(rodrig_mat3axis(tilt_axis2,tilt_angle2));
    second_rot_mat2inv(1:3,1:3) = transpose(rodrig_mat3axis(second_tilt_axis2,second_tilt_angle2));

    mat_a = rot_mat2inv*trans_mat('z',-rho1)*rot_mat1inv*trans_mat('x',-a);
    mat_b = second_rot_mat2inv*trans_mat('z',-rho2)*second_rot_mat1inv*trans_mat('y',-a);

    new_vect = mat_a*origin_mobile;
    second_new_vect = mat_b*origin_mobile;

    if h == 0 && 0 == 0
        theta13 = alpha;
        theta23 = beta;
    else
        sin_theta13 = (new_vect(2)*h - new_vect(3)*0)/(h^2 + 0^2);
        cos_theta13 = (-new_vect(2)*0 - new_vect(3)*h)/(h^2 + 0^2);

        sin_theta23 = (-second_new_vect(1)*h + second_new_vect(3)*0)/(h^2 + 0^2);
        cos_theta23 = (-second_new_vect(1)*0 - second_new_vect(3)*h)/(h^2 + 0^2);

        theta13 = atan2(sin_theta13,cos_theta13);
        theta23 = atan2(sin_theta23,cos_theta23);
    end
    
    %Passive universal joint check
     
    R_u11 = rodrig_mat3axis(tilt_axis, tilt_angle);
    rot_z_vec = R_u11*[0;0;1];
    rot_x_vec = R_u11*[1;0;0];
    
    ball_halfwidth_uni = sind(limits(1));
    if (abs(rot_z_vec(2)) > ball_halfwidth_uni) || (abs(rot_x_vec(2)) > ball_halfwidth_uni)
        p_lim_uni(1) = 0;
    end

    R_u21 = rodrig_mat3axis(second_tilt_axis, second_tilt_angle);
    rot_z_vec = R_u21*[0;0;1];
    rot_y_vec = R_u21*[0;1;0];

    if (abs(rot_z_vec(1)) > ball_halfwidth_uni) || (abs(rot_y_vec(1)) > ball_halfwidth_uni)
        p_lim_uni(2) = 0;
    end
    
    %Passive spherical joint check for universal joint

    ball_halfwidth_sph = sind(limits(2));
    
    R_s12 = rodrig_mat3axis(tilt_axis2, tilt_angle2);
    rot_z_vec = R_s12*[0;0;1];
    rot_x_vec = R_s12*[1;0;0];
    
    if (abs(rot_z_vec(2)) > ball_halfwidth_sph) || (abs(rot_x_vec(2)) > ball_halfwidth_sph)
        p_lim_sph(1) = 0;
    end
    
    R_s22 = rodrig_mat3axis(second_tilt_axis2, second_tilt_angle2);
    rot_z_vec = R_s22*[0;0;1];
    rot_y_vec = R_s22*[0;1;0];

    if (abs(rot_z_vec(1)) > ball_halfwidth_sph) || (abs(rot_y_vec(1)) > ball_halfwidth_sph)
        p_lim_sph(2) = 0;
    end
    
end
