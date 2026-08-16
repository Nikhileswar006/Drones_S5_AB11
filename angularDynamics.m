function dwdt = angularDynamics(w, J, tau)
% ANGULARDYNAMICS Computes rotational angular acceleration dw/dt (Eq. 9).
    w = w(:); tau = tau(:);
    gyroscopic_moment = cross(w, J * w);
    dwdt = J \ (-gyroscopic_moment + tau);
end
