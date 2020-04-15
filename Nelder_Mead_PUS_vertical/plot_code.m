function [] = plot_code(S_min)

    parameters = S_min; % [a, a_prime, h, t, offset]

    a = parameters(1);
    a_prime = parameters(2);
    h = parameters(3);
    t = parameters(4);
    offset = parameters(5);

    figure(1)
    subplot(2,2,1)
    alpha = 1;
    beta = 1;
    [u12, u21, s12, s22, h_point, t_point, x_plus, x_minus, y_plus, y_minus, offset1, offset2] = plot_para(alpha, beta, a, a_prime, h, t, offset);
    plot3([0,offset1(1)],[0,offset1(2)],[0,offset1(3)], '-b', 'LineWidth', 2);
    title('Plot for alpha = 1, beta = 1');
    hold on
    plot3([offset1(1),u12(1)],[offset1(2),u12(2)],[offset1(3),u12(3)], '-.r', 'LineWidth', 2);
     plot3([0,offset2(1)],[0,offset2(2)],[0,offset2(3)], '-b', 'LineWidth', 2);
    plot3([offset2(1),u21(1)],[offset2(2),u21(2)],[offset2(3),u21(3)], '-.k', 'LineWidth', 2);
    plot3([0,t_point(1)],[0,t_point(2)],[0,t_point(3)], '-b', 'LineWidth', 2);
    plot3([u12(1),s12(1)],[u12(2),s12(2)],[u12(3),s12(3)], '-r', 'LineWidth', 2);
    plot3([s12(1),h_point(1)],[s12(2),h_point(2)],[s12(3),h_point(3)], '-r', 'LineWidth', 2);
    plot3([u21(1),s22(1)],[u21(2),s22(2)],[u21(3),s22(3)], '-k', 'LineWidth', 2);
    plot3([s22(1),h_point(1)],[s22(2),h_point(2)],[s22(3),h_point(3)], '-k', 'LineWidth', 2);
    plot3([t_point(1),h_point(1)],[t_point(2),h_point(2)],[t_point(3),h_point(3)], '-g', 'LineWidth', 2);
    plot3([x_plus(1),x_minus(1)],[x_plus(2),x_minus(2)],[x_plus(3),x_minus(3)], '-m', 'LineWidth', 3);
    plot3([y_plus(1),y_minus(1)],[y_plus(2),y_minus(2)],[y_plus(3),y_minus(3)], '-m', 'LineWidth', 3);
    hold off;
    %axis([-1 a+0.3 -a-0.8 a+0.3 -0.2 h+t+0.5]);
    view(-190,30);

    subplot(2,2,2)
    [u12, u21, s12, s22, h_point, t_point, x_plus, x_minus, y_plus, y_minus, offset1, offset2] = plot_para(alpha, beta, a, a_prime, h, t, offset);
    plot3([0,offset1(1)],[0,offset1(2)],[0,offset1(3)], '-b', 'LineWidth', 2);
    title('Plot for alpha = 1, beta = 1');
    hold on
    plot3([offset1(1),u12(1)],[offset1(2),u12(2)],[offset1(3),u12(3)], '-.r', 'LineWidth', 2);
     plot3([0,offset2(1)],[0,offset2(2)],[0,offset2(3)], '-b', 'LineWidth', 2);
    plot3([offset2(1),u21(1)],[offset2(2),u21(2)],[offset2(3),u21(3)], '-.k', 'LineWidth', 2);
    plot3([0,t_point(1)],[0,t_point(2)],[0,t_point(3)], '-b', 'LineWidth', 2);
    plot3([u12(1),s12(1)],[u12(2),s12(2)],[u12(3),s12(3)], '-r', 'LineWidth', 2);
    plot3([s12(1),h_point(1)],[s12(2),h_point(2)],[s12(3),h_point(3)], '-r', 'LineWidth', 2);
    plot3([u21(1),s22(1)],[u21(2),s22(2)],[u21(3),s22(3)], '-k', 'LineWidth', 2);
    plot3([s22(1),h_point(1)],[s22(2),h_point(2)],[s22(3),h_point(3)], '-k', 'LineWidth', 2);
    plot3([t_point(1),h_point(1)],[t_point(2),h_point(2)],[t_point(3),h_point(3)], '-g', 'LineWidth', 2);
    plot3([x_plus(1),x_minus(1)],[x_plus(2),x_minus(2)],[x_plus(3),x_minus(3)], '-m', 'LineWidth', 3);
    plot3([y_plus(1),y_minus(1)],[y_plus(2),y_minus(2)],[y_plus(3),y_minus(3)], '-m', 'LineWidth', 3);
    hold off;
    view(135,40)

    subplot(2,2,3)
    alpha = -1;
    beta = 1;
    [u12, u21, s12, s22, h_point, t_point, x_plus, x_minus, y_plus, y_minus, offset1, offset2] = plot_para(alpha, beta, a, a_prime, h, t, offset);
    plot3([0,offset1(1)],[0,offset1(2)],[0,offset1(3)], '-b', 'LineWidth', 2);
    title('Plot for alpha = 1, beta = 1');
    hold on
    plot3([offset1(1),u12(1)],[offset1(2),u12(2)],[offset1(3),u12(3)], '-.r', 'LineWidth', 2);
     plot3([0,offset2(1)],[0,offset2(2)],[0,offset2(3)], '-b', 'LineWidth', 2);
    plot3([offset2(1),u21(1)],[offset2(2),u21(2)],[offset2(3),u21(3)], '-.k', 'LineWidth', 2);
    plot3([0,t_point(1)],[0,t_point(2)],[0,t_point(3)], '-b', 'LineWidth', 2);
    plot3([u12(1),s12(1)],[u12(2),s12(2)],[u12(3),s12(3)], '-r', 'LineWidth', 2);
    plot3([s12(1),h_point(1)],[s12(2),h_point(2)],[s12(3),h_point(3)], '-r', 'LineWidth', 2);
    plot3([u21(1),s22(1)],[u21(2),s22(2)],[u21(3),s22(3)], '-k', 'LineWidth', 2);
    plot3([s22(1),h_point(1)],[s22(2),h_point(2)],[s22(3),h_point(3)], '-k', 'LineWidth', 2);
    plot3([t_point(1),h_point(1)],[t_point(2),h_point(2)],[t_point(3),h_point(3)], '-g', 'LineWidth', 2);
    plot3([x_plus(1),x_minus(1)],[x_plus(2),x_minus(2)],[x_plus(3),x_minus(3)], '-m', 'LineWidth', 3);
    plot3([y_plus(1),y_minus(1)],[y_plus(2),y_minus(2)],[y_plus(3),y_minus(3)], '-m', 'LineWidth', 3);
    hold off;
    view(-200,30);

    subplot(2,2,4)
    alpha = 1;
    beta = -1;
    [u12, u21, s12, s22, h_point, t_point, x_plus, x_minus, y_plus, y_minus, offset1, offset2] = plot_para(alpha, beta, a, a_prime, h, t, offset);
    plot3([0,offset1(1)],[0,offset1(2)],[0,offset1(3)], '-b', 'LineWidth', 2);
    title('Plot for alpha = 1, beta = 1');
    hold on
    plot3([offset1(1),u12(1)],[offset1(2),u12(2)],[offset1(3),u12(3)], '-.r', 'LineWidth', 2);
     plot3([0,offset2(1)],[0,offset2(2)],[0,offset2(3)], '-b', 'LineWidth', 2);
    plot3([offset2(1),u21(1)],[offset2(2),u21(2)],[offset2(3),u21(3)], '-.k', 'LineWidth', 2);
    plot3([0,t_point(1)],[0,t_point(2)],[0,t_point(3)], '-b', 'LineWidth', 2);
    plot3([u12(1),s12(1)],[u12(2),s12(2)],[u12(3),s12(3)], '-r', 'LineWidth', 2);
    plot3([s12(1),h_point(1)],[s12(2),h_point(2)],[s12(3),h_point(3)], '-r', 'LineWidth', 2);
    plot3([u21(1),s22(1)],[u21(2),s22(2)],[u21(3),s22(3)], '-k', 'LineWidth', 2);
    plot3([s22(1),h_point(1)],[s22(2),h_point(2)],[s22(3),h_point(3)], '-k', 'LineWidth', 2);
    plot3([t_point(1),h_point(1)],[t_point(2),h_point(2)],[t_point(3),h_point(3)], '-g', 'LineWidth', 2);
    plot3([x_plus(1),x_minus(1)],[x_plus(2),x_minus(2)],[x_plus(3),x_minus(3)], '-m', 'LineWidth', 3);
    plot3([y_plus(1),y_minus(1)],[y_plus(2),y_minus(2)],[y_plus(3),y_minus(3)], '-m', 'LineWidth', 3);
    hold off;
    view(135,40);

figure(2)
    alpha = 0;
    beta = 0;
    [u12, u21, s12, s22, h_point, t_point, x_plus, x_minus, y_plus, y_minus, offset1, offset2] = plot_para(alpha, beta, a, a_prime, h, t, offset);
    plot3([0,offset1(1)],[0,offset1(2)],[0,offset1(3)], '-b', 'LineWidth', 2);
    title('Plot for alpha = 1, beta = 1');
    hold on
    plot3([offset1(1),u12(1)],[offset1(2),u12(2)],[offset1(3),u12(3)], '-.r', 'LineWidth', 2);
     plot3([0,offset2(1)],[0,offset2(2)],[0,offset2(3)], '-b', 'LineWidth', 2);
    plot3([offset2(1),u21(1)],[offset2(2),u21(2)],[offset2(3),u21(3)], '-.k', 'LineWidth', 2);
    plot3([0,t_point(1)],[0,t_point(2)],[0,t_point(3)], '-b', 'LineWidth', 2);
    plot3([u12(1),s12(1)],[u12(2),s12(2)],[u12(3),s12(3)], '-r', 'LineWidth', 2);
    plot3([s12(1),h_point(1)],[s12(2),h_point(2)],[s12(3),h_point(3)], '-r', 'LineWidth', 2);
    plot3([u21(1),s22(1)],[u21(2),s22(2)],[u21(3),s22(3)], '-k', 'LineWidth', 2);
    plot3([s22(1),h_point(1)],[s22(2),h_point(2)],[s22(3),h_point(3)], '-k', 'LineWidth', 2);
    plot3([t_point(1),h_point(1)],[t_point(2),h_point(2)],[t_point(3),h_point(3)], '-g', 'LineWidth', 2);
    plot3([x_plus(1),x_minus(1)],[x_plus(2),x_minus(2)],[x_plus(3),x_minus(3)], '-m', 'LineWidth', 3);
    plot3([y_plus(1),y_minus(1)],[y_plus(2),y_minus(2)],[y_plus(3),y_minus(3)], '-m', 'LineWidth', 3);
    hold off;
    view(-200,30)

end
