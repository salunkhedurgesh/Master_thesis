function check = try_break(i)
check = 7;


    fprintf("Something \n");
    if i == 6
        fprintf("I was here\n");
        check = 5;
        return;
    end
