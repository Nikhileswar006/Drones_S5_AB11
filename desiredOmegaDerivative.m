function dw_err_d = desiredOmegaDerivative(q_err, w_err, w_ref, K1)
% DESIREDOMEGADERIVATIVE Computes derivative dw_err_d/dt = -K1 * deps_err/dt (Eq. 15).
    q0 = q_err(1);
    eps = q_err(2:4);
    eps = eps(:);
    w_err = w_err(:);
    w_ref = w_ref(:);
    S_eps = [   0,     -eps(3),  eps(2);
              eps(3),    0,     -eps(1);
             -eps(2),  eps(1),    0    ];
    deps_err = 0.5 * (q0 * eye(3) + S_eps) * w_err + cross(eps, w_ref);
    dw_err_d = -K1 * deps_err;
end
