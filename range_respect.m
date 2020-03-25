function return_point = range_respect(in_point,n)
    
a_range = [0.25,1.25];
a_prime_range = [0.25,2];
h_range = [-0.5,1.5];
t_range = [1,4];
    
for i = 1:n+1
        if in_point(1) < a_range(1)
            in_point(1) = a_range(1);
        elseif in_point(1) > a_range(2)
            in_point(1) = a_range(2);
        end
        if in_point(2) < a_prime_range(1)
            in_point(2) = a_prime_range(1);
        elseif in_point(2) > a_prime_range(2)
            in_point(2) = a_prime_range(2);
        end
        if in_point(3) < h_range(1)
            in_point(3) = h_range(1);
        elseif in_point(3) > h_range(2)
            in_point(3) = h_range(2);
        end
        if in_point(4) < t_range(1)
            in_point(4) = t_range(1);
        elseif in_point(4) > t_range(2)
            in_point(4) = t_range(2);
        end
end
return_point = in_point;
end