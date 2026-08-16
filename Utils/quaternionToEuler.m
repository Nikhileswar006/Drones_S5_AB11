function [phi, theta, psi] = quaternionToEuler(q)
% QUATERNIONTOEULER Converts unit quaternion [q0, q1, q2, q3] to Euler angles (rad).
    q = q(:);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    sinr_cosp = 2 * (qw * qx + qy * qz);
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    phi = atan2(sinr_cosp, cosr_cosp);
    sinp = 2 * (qw * qy - qz * qx);
    if abs(sinp) >= 1
        theta = sign(sinp) * (pi / 2);
    else
        theta = asin(sinp);
    end
    siny_cosp = 2 * (qw * qz + qx * qy);
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    psi = atan2(siny_cosp, cosy_cosp);
end
