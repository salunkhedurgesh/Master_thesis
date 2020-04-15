function S = sobol_tuning(n,a_range,a_prime_range,h_range,t_range,offset_range,val)

%% Stage 1: Choosing the Simplex
%Dimension of a point in simplex = 4 (a, a_prime, h and t)
%We will use sobolset for generating points for multi_starting the simplex
%n = 4; %number of dimensions
p = sobolset(n,'Skip',1e3,'Leap',1e2);
p = scramble(p,'MatousekAffineOwen');
fprintf('Generating points using sobol function \n');
%S has valid points each of dimension 4
S = zeros(val,n);

%This loop is because the sobolset generates many points from
%the given range of the parameters but we want those points that have no
%singularity curves in the RDW of +- 1 radian square defined in the output
%workspace parmetrized by alpha and beta
i = 1;
j = 1;
while(j<=val)
a = a_range(1) + p(i,1)*(a_range(2) - a_range(1));
a_prime = a_prime_range(1) + p(i,2)*(a_prime_range(2) - a_prime_range(1));
h = h_range(1) + p(i,3)*(h_range(2) - h_range(1));
t = t_range(1) + p(i,4)*(t_range(2) - t_range(1));
offset = offset_range(1) + p(i,5)*(offset_range(2) - offset_range(1));

sing_bool = RDW_sing(a, a_prime, h, t, offset);
    if sing_bool == 1
        S(j,1:n) = [a,a_prime,h,t,offset];
        j = j+1;
    end
i = i+1;

end
end
