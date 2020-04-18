% Description: Generating the initial simplexes for all the multi starts
% Inputs:
% 1. type of the mechanism
% 2. ranges of the parameters in matrix of (n x 2)
% 3. number of multi starts
% None
% Outpus:
% 1. Initial simplex for all multi starts

function S = rand_tuning(type, ranges, starts)
    
    n = size(ranges, 1);
    val = (n+1)*starts;
    p = rand(val*100, n);
    fprintf('Generating points using random function \n');
    
    S = zeros(val,n);
    
    % This loop is because the random function generates many points from
    % the given range of the parameters but we want those points that have no
    % singularity curves in the RDW of +- 1 radian square defined in the output
    % workspace parmetrized by alpha and beta
    i = 1;
    j = 1;
    par_inst = zeros(1,n);
    while(j<=val)
        for par = 1:n
            par_inst(par) = ranges(par, 1) + p(i, par)*(ranges(par, 2) - ranges(par, 1));
        end
        
        sing_bool = RDW_sing(type, par_inst);
        if sing_bool == 1
            S(j,:) = par_inst;
            j = j+1;
        end
        i = i+1;
        if i > size(p(:,1))
            error('unable to generate the valid points, consider:\n1. Giving better ranges of the parameters\n2. Increasing number of random points generated in function file rand_tuning.m\n');
        end
        
    end
end
