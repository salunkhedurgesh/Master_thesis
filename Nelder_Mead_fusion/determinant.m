% function name: determinant
% Description: Calculates the determinant and saves its symbolic expression
% Inputs: type of the mechanism (2UPS or 2PUS)
% Outpus: symbolic determinant

function det_sym= determinant(type)
    
    if type == "2UPS"
        det_sym = determinant_UPS();
    elseif type == "2PUS"
        det_sym = determinant_PUS();
    end
    
end


