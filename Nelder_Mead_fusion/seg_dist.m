% seg1 -> 2x3 matrix representing 2 points in 3d
% seg2 -> 2x3 matrix representing 2 points in 3d

function dist = seg_dist(seg1, seg2, rho_min)
    
% seg1 = [u11x, u11y, u11z; 
%         s12x, s12y, s12z];
%     
% seg2 = [u21x, u21y, u21z; 
%         s22x, s22y, s22z];
    iter = 1;
    step = 5;
    found1 = 0;
    found2 = 0;
    for i = 0:step
        for j = 0:step
            for k = 1:3
                point_seg1(k) = seg1(1, k) + (seg1(2, k) - seg1(1, k))*i/step;
                if found1 == 0
                    if norm(point_seg1(k) - point_seg1(1)) > rho_min
                        found1 = k;
                    end
                end                        
                point_seg2(k) = seg2(1, k) + (seg2(2, k) - seg2(1, k))*j/step;
                if found2 == 0
                    if norm(point_seg2(k) - point_seg2(1)) > rho_min
                        found2 = k;
                    end
                end
            end
            
            dist_vec(iter) = norm(point_seg1-point_seg2, 2);
            if found1 ~= 0 && found2 ~=0 && i >= max(found1, found2) && j >= max(found1, found2) 
                dist_vec(iter) = (5 * dist_vec(iter))/3;
            end
            iter = iter + 1;
        end
    end
    
    dist = min(dist_vec);
end