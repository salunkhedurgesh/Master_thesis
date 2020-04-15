function return_point = range_respect(in_point,a_range,a_prime_range,h_range,t_range,n)
    
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