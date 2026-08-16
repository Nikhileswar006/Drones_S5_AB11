function w_err_d = desiredOmegaError(q_err, K1)
% DESIREDOMEGAERROR Computes virtual control law w_err_d = -K1 * eps_err (Eq. 13).
    eps_err = q_err(2:4);
    eps_err = eps_err(:);
    w_err_d = -K1 * eps_err;
end
