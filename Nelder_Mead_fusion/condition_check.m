syms l1 l2 theta1 theta2;
J = eye(2);
j11 = -l1 * sin(theta1) - l2 * sin(theta1 + theta2);
j12 = - l2 * sin(theta1 + theta2); 
j21 = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
j22 = l2 * cos(theta1 + theta2);