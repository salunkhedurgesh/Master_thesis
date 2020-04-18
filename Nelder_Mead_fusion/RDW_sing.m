% Description: check if there lies a singularity in the Regular Dextrous
%Workspace (RDW)
% Inputs:
% 1. type of the mechanism
% 2. parameters
% None
% Outpus:
% 1. boolean for non singular RDW
function sing_bool = RDW_sing(type, parameters)
    
    if type == "2UPS"
        sing_bool = RDW_sing_UPS(parameters);
    elseif type == "2PUS"
        sing_bool = RDW_sing_PUS(parameters);
    end
    
    
end
