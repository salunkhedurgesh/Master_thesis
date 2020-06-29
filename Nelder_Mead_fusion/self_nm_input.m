% function name: self_nm_input
% Description: The information required for the definition of the optimisation
% Inputs: None
% Outpus:
% 1. Dimension of the points in optimisation
% 2. The matrix related to the ranges of the parameters
% 3. The number of starts for multi-starting the Nelder Mead process
% 4. Number of iterations to be continued in case of encountering the same solution
% 5. The vector for passive limits of U-joint, S-joint and the ratio for
% the actuator stroke.
% 6. The choice of objective function
% 7. Availability of the sobolset function (to generate low discrepancy points)
% 8. filenames where the data is stored for the optimisation process

function [type, n, ranges, starts, iterations, limits, objective_choice, sobol_var, save_files, reward, maximize] = self_nm_input()

type = "2UPS";
n = 13;
% [u11x, u11y, u11z, s12x, s12y, s12z, u21x, u21y, u21z, s22x, s22y, s22z, t]
%     ranges = [0.25, 1.5;  %range of radius of universal joint of first leg
%               -pi/2-deg2rad(10), pi/2+deg2rad(10);  %range of theta of universal joint of first leg
%               -0.1, 0.1;  %range of z co-ordinate of universal joint of first leg
%               0.25, 2;    %range of radius of spherical joint of first leg
%               -pi/2-deg2rad(10), pi/2+deg2rad(10);    %range of theta of spherical joint of first leg
%               -0.5, 0.5;  %range of z co-ordinate of spherical joint of first leg
%               0.25, 1.5;  %range of radius of universal joint of second leg
%               -pi/2-deg2rad(10), pi/2+deg2rad(10);  %range of theta of universal joint of second leg
%               -0.1, 0.1;  %range of z co-ordinate of universal joint of second leg
%               0.25, 2;    %range of radius of spherical joint of second leg
%               -pi/2-deg2rad(10), pi/2+deg2rad(10);    %range of theta of spherical joint of second leg
%               -0.5, 0.5;  %range of z co-ordinate of spherical joint of second leg
%                1, 4];     %range of t
ranges = [0.25, 1.5;
    -pi/2-deg2rad(10), pi/2+deg2rad(10);
    -0.1, 0.1;
    0.25, 2;
    -pi/2-deg2rad(10), pi/2+deg2rad(10);
    -0.5, 0.5;
    0.25, 1.5;
    -pi/2-deg2rad(10), pi/2+deg2rad(10);
    -0.1, 0.1;
    0.25, 2;
    -pi/2-deg2rad(10), pi/2+deg2rad(10);
    -0.5, 0.5;
    1, 4];
starts = 2;
iterations = 25;
limits = [45, 45, 1.5, 0.3]; %[universal_joint, spherical_joint, stroke_ratio, minimum_quality_index]
objective_choice = "workspace";
sobol_var = 1;
format shortg
save_files = record_file();
reward = "binary";
maximize = "inner_conditioning";%"joint_quality"; %"conditioning_number";
format shortg
fprintf('The mechanism is of type %s with dimension %d\nThe number of starts are %d\n', type, n, starts);
fprintf('The number of iterations to continue for same solution are %d \n', iterations);
fprintf('Universal limit = %d degrees, Spherical limits = %d degrees, ratio of actuator stroke = %0.2f \n', limits(1), limits(2), limits(3));
fprintf('The mechanism is being optimised for %s with %s rewarding function\n', objective_choice, reward);
fprintf('The %s is also being maximized along with the workspace \n', maximize);
if sobol_var == 0
    fprintf('Sobolset functionality is not used \n');
else
    fprintf('Sobolset functionality is used to generate low discrepancy points for multi start \n');
end
c = clock;
foldername = num2str(c(3)) + "_" + num2str(c(2));
fprintf('The file names are %s and %s saved in %s folder. Both are opened in write mode \n', save_files(1), save_files(2), foldername);
end