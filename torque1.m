function tau1 = torque1(w, J, dw_ref, dw_err_d, q_err)
% TORQUE1 Computes non-linear feedback cancellation torque tau1 (Eq. 21).
    w = w(:);
    dw_ref = dw_ref(:);
    dw_err_d = dw_err_d(:);
    eps_err = q_err(2:4);
    eps_err = eps_err(:);
    gyroscopic = cross(w, J * w);
    tau1 = gyroscopic + J * dw_ref + J * dw_err_d - eps_err;
end
