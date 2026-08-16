function w_err = angularVelocityError(w, w_ref)
% ANGULARVELOCITYERROR Computes angular velocity error w_err = w - w_ref (Eq. 11).
    w_err = w(:) - w_ref(:);
end
