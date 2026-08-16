function dqdt = quaternionDynamics(q, w)
% QUATERNIONDYNAMICS Computes kinematic quaternion derivative dq/dt (Eq. 8).
    q = q(:); w = w(:);
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    wx = w(1); wy = w(2); wz = w(3);
    dq0 = 0.5 * (-q1*wx - q2*wy - q3*wz);
    dq1 = 0.5 * ( q0*wx - q3*wy + q2*wz);
    dq2 = 0.5 * ( q3*wx + q0*wy - q1*wz);
    dq3 = 0.5 * (-q2*wx + q1*wy + q0*wz);
    dqdt = [dq0; dq1; dq2; dq3];
end
