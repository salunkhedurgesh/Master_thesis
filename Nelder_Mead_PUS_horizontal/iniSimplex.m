function [sobol_set, suc_var] = iniSimplex(n, starts, sobol_var, ranges)
    
    a_range = ranges(1,:);
    a_prime_range = ranges(2,:);
    h_range = ranges(3,:);
    t_range = ranges(4,:);
    valid_points = n*starts; %Number of starts = valid_points/5
    fprintf('Please wait: Calculating the set of valid points for a initial simplex point that is non-singular in the desired RDW \n')
    if sobol_var == 0
        [sobol_set, suc_var] = rand_tuning(n,a_range,a_prime_range,h_range,t_range,valid_points);
    else
        [sobol_set, suc_var] = sobol_tuning(n,a_range,a_prime_range,h_range,t_range,valid_points);
    end
end
