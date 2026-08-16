function q_inv = quatInverse(q)
% QUATINVERSE Computes quaternion inverse q^-1.
    q_conj = quatConjugate(q);
    q_inv = q_conj / (norm(q)^2);
end
