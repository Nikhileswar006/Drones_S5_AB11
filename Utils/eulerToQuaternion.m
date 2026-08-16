function q = eulerToQuaternion(phi, theta, psi)
% EULERTOQUATERNION Converts Euler angles (phi, theta, psi in rad) to unit quaternion.
    c1 = cos(phi/2); s1 = sin(phi/2);
    c2 = cos(theta/2); s2 = sin(theta/2);
    c3 = cos(psi/2); s3 = sin(psi/2);
    qw = c1*c2*c3 + s1*s2*s3;
    qx = s1*c2*c3 - c1*s2*s3;
    qy = c1*s2*c3 + s1*c2*s3;
    qz = c1*c2*s3 - s1*s2*c3;
    q = [qw; qx; qy; qz];
    q = q / norm(q);
end
