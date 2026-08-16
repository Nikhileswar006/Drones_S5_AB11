function q_err = quaternionError(q_ref, q)
% QUATERNIONERROR Computes attitude quaternion error q_err = q_ref* (x) q (Eq. 10).
    q_ref_conj = quatConjugate(q_ref);
    q_err = quatMultiply(q_ref_conj, q);
    q_err = quatNormalize(q_err);
    if q_err(1) < 0
        q_err = -q_err;
    end
end
