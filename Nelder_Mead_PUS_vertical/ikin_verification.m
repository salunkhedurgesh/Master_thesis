clear all;
close all;
clc

a = 0.5;
a_prime = 0.25;
h = 0;
t = 0.25;
offset = 0.1;
ball_halfwidth = sind(45);
ik_iter = 1;
    hk = 0;
    abs_x = [1;0;0];
    abs_y = [0;1;0];
    abs_z = [0;0;1];
    %origin = [0;0;0];
    origin_mobile = [0;0;t;1];
    %u11 = [a;0;0];
    s12_mobile = [a_prime;hk;h];
    %u21 = [0;a;0];
    s22_mobile = [hk;a_prime;h];
for alpha = -1 :0.01:1
    for beta = -1 :0.01:1
    fixed_T_mobile = trans_mat('z', t)*rot_mat('x', alpha)*rot_mat('y', beta);
    s12 = fixed_T_mobile*[s12_mobile;1];
    s22 = fixed_T_mobile*[s22_mobile;1];

    rho1 = s12(3) + sqrt(a^2 - s12(2)^2 - (offset - s12(1))^2);
    rho2 = s22(3) + sqrt(a^2 - s22(1)^2 - (offset - s22(2))^2);
    
    u11_T_fixed = inv(trans_mat('x', offset)*trans_mat('z',rho1));
    u21_T_fixed = inv(trans_mat('y', offset)*trans_mat('z',rho2));

    s12_u11 = u11_T_fixed*s12;
    s22_u21 = u21_T_fixed*s22;
    
    u11 = trans_mat('x', offset)*trans_mat('z',rho1)*[0;0;0;1];
    u21 = trans_mat('x', offset)*trans_mat('z',rho1)*[0;0;0;1];
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

    sub_vect = fixed_T_mobile*[0;hk;h;1];
    second_sub_vect = fixed_T_mobile*[hk;0;h;1];

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

    mat_a = rot_mat2inv*trans_mat('z',-a)*rot_mat1inv*trans_mat('z',-rho1)*trans_mat('x',-offset);
    mat_b = rot_mat2inv*trans_mat('z',-a)*rot_mat1inv*trans_mat('z',-rho2)*trans_mat('y',-offset);

    new_vect = mat_a*origin_mobile;
    second_new_vect = mat_b*origin_mobile;

    sin_theta13 = (new_vect(2)*h - new_vect(3)*hk)/(h^2 + hk^2);
    cos_theta13 = (-new_vect(2)*hk - new_vect(3)*h)/(h^2 + hk^2);

    sin_theta23 = (-second_new_vect(1)*h + second_new_vect(3)*hk)/(h^2 + hk^2);
    cos_theta23 = (-second_new_vect(1)*hk - second_new_vect(3)*h)/(h^2 + hk^2);

    theta13 = atan2(sin_theta13,cos_theta13);
    theta23 = atan2(sin_theta23,cos_theta23);
    
    if h == 0 && hk == 0
        theta13 = alpha;
        theta23 = beta;
    end
    
    %Passive universal joint check
    p_lim_uni = 1;
    p_lim_sph = 0;
    
    R_u11 = rodrig_mat3axis(tilt_axis, tilt_angle);
    rot_z_vec = R_u11*[0;0;1];
    rot_x_vec = R_u11*[1;0;0];       
        
    recheck_ikin = trans_mat('x',offset)*trans_mat('z',rho1)*rodrig_mat4axis(tilt_axis,tilt_angle)*trans_mat('z',a)*rodrig_mat4axis(tilt_axis2,tilt_angle2)*rot_mat('x',theta13)*[-a_prime;0;-h;1];
%     recheck_ikin = trans_mat('x',offset)*trans_mat('z',rho1)*rodrig_mat4axis(tilt_axis,tilt_angle)*trans_mat('z',a)*[0;0;0;1];
    cor_vec(ik_iter,:) = recheck_ikin - [0;0;t;1];
    ik_iter = ik_iter +1;
    end
end