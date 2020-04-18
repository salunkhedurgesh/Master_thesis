% function name: filecheck()
% description : Checks fo rerrors in opening the file
% input : file descriptor
% output : Prints on screen the result

function [] = filecheck(fd)
    if fd < 0
        fprintf('Error in opening file');
    else
        [~,permission,~,~] = fopen(fd);
        if permission == 'rb'
            per_string = 'reading';
        elseif permission == 'wb'
            per_string = 'writing';
        elseif permission == 'ab'
            per_string = 'appending';
        end
        fprintf('Sucessfully opened the file in %s mode\n', per_string);
    end
end