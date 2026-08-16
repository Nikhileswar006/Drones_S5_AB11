function q_conj = quatConjugate(q)
% QUATCONJUGATE Computes quaternion conjugate q* = [q0; -q1; -q2; -q3].
    q = q(:);
    q_conj = [q(1); -q(2); -q(3); -q(4)];
end
