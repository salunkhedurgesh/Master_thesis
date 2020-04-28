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
    
    u11x = parameters(1);
    u11y= parameters(2);
    u11z = parameters(3);
    
    s12x = parameters(4);
    s12y= parameters(5);
    s12z = parameters(6);
    
    u21x = parameters(7);
    u21y= parameters(8);
    u21z = parameters(9);
    
    s22x = parameters(10);
    s22y= parameters(11);
    s22z = parameters(12);
    
    t = parameters(13);
    
    p_lim_uni = [1, 1];
    p_lim_sph = [1, 1];
    rho_inst = [0, 0];
    
    determinant = ((2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*(s22y*cos(alpha) - s22z*cos(beta)*sin(alpha) + s22x*sin(alpha)*sin(beta)) + 2*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*(s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta)))*(2*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*(s12x*cos(beta)*sin(alpha) + s12z*sin(alpha)*sin(beta)) + 2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*(s12x*cos(alpha)*cos(beta) + s12z*cos(alpha)*sin(beta)) - 2*abs(s12x*cos(beta) - u11x + s12z*sin(beta))*sign(s12x*cos(beta) - u11x + s12z*sin(beta))*(s12z*cos(beta) - s12x*sin(beta))))/(4*(abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))^2)^(1/2)*(abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))^2)^(1/2)) - ((2*abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*sign(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))*(s12y*cos(alpha) - s12z*cos(beta)*sin(alpha) + s12x*sin(alpha)*sin(beta)) + 2*abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*sign(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))*(s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta)))*(2*abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*sign(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))*(s22x*cos(beta)*sin(alpha) + s22z*sin(alpha)*sin(beta)) + 2*abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*sign(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))*(s22x*cos(alpha)*cos(beta) + s22z*cos(alpha)*sin(beta)) - 2*abs(s22x*cos(beta) - u21x + s22z*sin(beta))*sign(s22x*cos(beta) - u21x + s22z*sin(beta))*(s22z*cos(beta) - s22x*sin(beta))))/(4*(abs(u11y - s12y*cos(alpha) + s12z*cos(beta)*sin(alpha) - s12x*sin(alpha)*sin(beta))^2 + abs(s12x*cos(beta) - u11x + s12z*sin(beta))^2 + abs(t - u11z + s12y*sin(alpha) + s12z*cos(alpha)*cos(beta) - s12x*cos(alpha)*sin(beta))^2)^(1/2)*(abs(u21y - s22y*cos(alpha) + s22z*cos(beta)*sin(alpha) - s22x*sin(alpha)*sin(beta))^2 + abs(s22x*cos(beta) - u21x + s22z*sin(beta))^2 + abs(t - u21z + s22y*sin(alpha) + s22z*cos(alpha)*cos(beta) - s22x*cos(alpha)*sin(beta))^2)^(1/2));
    det_var = "NS"; % non-singular
    if abs(determinant) < 0.005
        det_var = "singular";
        return;
    end
    
    abs_z = [0;0;1];
    origin_mobile = [0;0;t;1];
    s12_mobile = [s12x; s12y; s12z];
    s22_mobile = [s22x; s22y; s22z];

    fixed_T_mobile = trans_mat('z', t)*rot_mat('x', alpha)*rot_mat('y', beta);
    s12 = fixed_T_mobile*[s12_mobile;1];
    s22 = fixed_T_mobile*[s22_mobile;1];

    u11_T_fixed = trans_mat('x', -u11x)*trans_mat('y', -u11y)*trans_mat('z', -u11z);
    u21_T_fixed = trans_mat('x', -u21x)*trans_mat('y', -u21y)*trans_mat('z', -u21z);

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

    sub_vect = fixed_T_mobile*[0;0;s12z;1];
    second_sub_vect = fixed_T_mobile*[0;0;s22z;1];

    ee_x_s12 = [-s12x;-s12y;0];
    ee_y_s22 = [-s22x;-s22y;0];

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

    mat_a = rot_mat2inv*trans_mat('z',-rho1)*rot_mat1inv*(trans_mat('x', -u11x)*trans_mat('y', -u11y)*trans_mat('z', -u11z));
    mat_b = second_rot_mat2inv*trans_mat('z',-rho2)*second_rot_mat1inv*(trans_mat('x', -u21x)*trans_mat('y', -u21y)*trans_mat('z', -u21z));

    new_vect = mat_a*origin_mobile;
    second_new_vect = mat_b*origin_mobile;

    if s12y == 0 && s12z == 0
        theta13 = alpha;
        theta23 = beta;
    else
        sin_theta13 = (new_vect(2)*s12z - new_vect(3)*s12y)/(s12z^2 + s12y^2);
        cos_theta13 = (-new_vect(2)*s12y - new_vect(3)*s12z)/(s12z^2 + s12y^2);

        sin_theta23 = (-second_new_vect(1)*s22z + second_new_vect(3)*s22x)/(s22z^2 + s22x^2);
        cos_theta23 = (-second_new_vect(1)*s22x - second_new_vect(3)*s22z)/(s22z^2 + s22x^2);

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
