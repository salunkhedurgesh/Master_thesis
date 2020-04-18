% function name: determinant_PUS()
% Description: Calculates the determinant and saves its symbolic expression
% Inputs: None
% Outpus: symbolic determinant

function det_sym = determinant_PUS()
    syms a a_prime h t theta delta offset;
    s12_mobile = [a_prime;0;h];
    s22_mobile = [0;a_prime;h];
    
    fixed_T_mobile = trans_mat('z', t)*rot_mat('x', theta)*rot_mat('y', delta);
    s12 = fixed_T_mobile*[s12_mobile;1];
    s22 = fixed_T_mobile*[s22_mobile;1];
    
    
    
    sqrt_s12 = sqrt(a^2 - s12(2)^2 - (offset - s12(1))^2);
    sqrt_s22 = sqrt(a^2 - s22(1)^2 - (offset - s22(2))^2);
    fprintf('ATTENTION: There are 4 ikin solutions to this configuration\n');
    fprintf('Choosing the solutions of type [ 0.5*a ''+'' 0.5*sqrt(b^2 - 4ac) ] \n');
    rho1 = s12(3) + sqrt_s12;
    rho2 = s22(3) + sqrt_s22;
    
    
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