function delta = deltaError(w_err, w_err_d)
% DELTAERROR Computes backstepping auxiliary error delta = w_err - w_err_d (Eq. 16).
    delta = w_err(:) - w_err_d(:);
end
