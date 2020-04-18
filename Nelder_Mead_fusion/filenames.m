% function name: filenames()
% Description: Saving the filenames for further process
% Inputs: 
% None
% Outpus: 
% 1. All file names

function [save_files] = filenames()
    
    prompt_deep = 'file name for saving deep analysis (eg: ''deep.txt'')\n';
    file_ms_deep = input(prompt_deep);
    prompt_point_record = 'file name for saving deep analysis (eg: ''point_record.txt'')\n';
    file_ms_point = input(prompt_point_record);
    save_files = [string(file_ms_deep), string(file_ms_point)];
end