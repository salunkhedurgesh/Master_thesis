function return_point = range_respect(in_point,ranges,n)
    
    for i = 1:n
        if in_point(i) < ranges(i,1)
            in_point(i) = ranges(i,1);
        elseif in_point(i) > ranges(i,2)
            in_point(i) = ranges(i,2);
        end
    end
return_point = in_point;
end