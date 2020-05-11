function angle = tilt_angle(a, b)
    
a = a(1:3);
b = b(1:3);
angle = acos(dot(a,b)/(norm(a, 2) * norm(b, 2)));
    
end