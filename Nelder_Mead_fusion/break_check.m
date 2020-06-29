clear all
clc
for i = 1:10
    for j = 1:20
        if (j > 10 || i < 5) && mod(j, 2) == 0
            fprintf('THE VALUE OF i IS %d AND j IS %d\n', i, j);
        else
            fprintf('The value of i is %d and j is %d\n', i, j);
        end
    end
end

for k = 1:5
    fprintf("The value of k is %d \n", k);
end