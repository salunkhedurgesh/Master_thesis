% function name: determinant_UPS()
% Description: Calculates the determinant and saves its symbolic expression
% Inputs: None
% Outpus: symbolic determinant

function det_sym = determinant_UPS()
    syms u11x u11y u11z s12x s12y s12z u21x u21y u21z s22x s22y s22z h t theta delta;
    s12_mobile = [s12x; s12y; s12z];
    s22_mobile = [s22x; s22y; s22z];
    
    fixed_T_mobile = trans_mat('z', t)*rot_mat('x', theta)*rot_mat('y', delta);
    s12 = fixed_T_mobile*[s12_mobile;1];
    s22 = fixed_T_mobile*[s22_mobile;1];
    
    u11_T_fixed = trans_mat('x', -u11x)*trans_mat('y', -u11y)*trans_mat('z', -u11z);
    u21_T_fixed = trans_mat('x', -u21x)*trans_mat('y', -u21y)*trans_mat('z', -u21z);
    
    s12_u11 = u11_T_fixed*s12;
    s22_u21 = u21_T_fixed*s22;
    
    rho1 = norm(s12_u11(1:3),2);
    rho2 = norm(s22_u21(1:3),2);
    
    j11 = diff(rho1, theta);
    j12 = diff(rho1, delta);
    j21 = diff(rho2, theta);
    j22 = diff(rho2, delta);
    
    determinant = j11*j22 - j12*j21;
    
    % Saving the determinant
    f0 = fopen('determinant.txt', 'w');
    filecheck(f0);
    fprintf(f0, '%s', determinant);
    fclose(f0);
    
    f1 = fopen('determinant.txt', 'r');
    filecheck(f1);
    det_size = [1, inf];
    det_format = '%s';
    det_string = fscanf(f1, det_format, det_size);
    fclose(f1);
    
    det_string = strrep( det_string, 'theta', 'alpha');
    det_string = strrep( det_string, 'delta', 'beta');
    det_sym = str2sym(det_string);
end