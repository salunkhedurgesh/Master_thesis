function t = trans_mat(axis,value)
t = eye(4);
   if axis == 'x'
       t(1,4) = value;
   elseif axis == 'y'
       t(2,4) = value;
   elseif axis == 'z'
        t(3,4) = value;
   else
       fprintf('Invalid input - check the axis or the value');
   end
end
