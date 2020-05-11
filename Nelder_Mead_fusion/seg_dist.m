% seg1 -> 2x3 matrix representing 2 points in 3d
% seg2 -> 2x3 matrix representing 2 points in 3d

function dist = seg_dist(seg1, seg2)
    
% seg1 = [u1x, u1y, u1z; 
%         s1x, s1y, s1z];
    
% seg2 = [u2x, u2y, u2z; 
%         s2x, s2y, s2z];
    iter = 1;
    step = 5;
    for i = 0:step
        for j = 0:step
            for k = 1:3
                point_seg1(k) = seg1(1, k) + (seg1(2, k) - seg1(1, k))*i/step;
                point_seg2(k) = seg2(1, k) + (seg2(2, k) - seg2(1, k))*j/step;
            end
            
            dist_vec(iter) = norm(point_seg1-point_seg2, 2);
            iter = iter + 1;
        end
    end
    
    dist = min(dist_vec);
end