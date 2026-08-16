function q_norm = quatNormalize(q)
% QUATNORMALIZE Normalizes quaternion to unit magnitude ||q|| = 1.
    q = q(:);
    n = norm(q);
    if n < 1e-12
        q_norm = [1; 0; 0; 0];
    else
        q_norm = q / n;
    end
end
