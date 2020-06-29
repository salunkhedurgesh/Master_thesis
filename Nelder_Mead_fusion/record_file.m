function [save_files] = record_file()
    format shortg
    c = clock;
    month_list = ["Jan", "Feb", "mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
    mmonth = month_list(c(2));
    file1 = "record_deep_" + num2str(c(3)) + mmonth + "_" + num2str(c(4)) + "hrs_" + num2str(c(5)) + "min" + ".txt";
    file2 = "record_points_" + num2str(c(3)) + mmonth + "_" + num2str(c(4)) + "hrs_" + num2str(c(5)) + "min" + ".txt";
    save_files = [file1, file2]; 
end