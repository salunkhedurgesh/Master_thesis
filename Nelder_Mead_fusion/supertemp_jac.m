function J = supertemp_jac(l1, l2, theta1, theta2)
J = eye(2);
J(1,1) = -l1 * sin(theta1) - l2 * sin(theta1 + theta2);
J(1,2) = - l2 * sin(theta1 + theta2); 
J(2,1) = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
J(2,2) = l2 * cos(theta1 + theta2);

for alpha = 0:0.01:2*pi
    theta1dot = cos(alpha);
    theta2dot = sin(alpha);
    xvel = J(1,1) * theta1dot + J(1, 2) * theta2dot;
    yvel = J(2,1) * theta1dot + J(2, 2) * theta2dot;
    plot(xvel, yvel, '*b');
    hold on
end
end