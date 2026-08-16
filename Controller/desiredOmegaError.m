function w_err_d = desiredOmegaError(q_err, K1)
% DESIREDOMEGAERROR Computes virtual control law matching exact Paper Eq. (13).
%
% Implements Paper Equation (13):
%   [0; w_err_d] = -k1 * q_err* (x) (q_err - [1; 0; 0; 0])
%
% Description:
%   Performs full 4D quaternion multiplication of q_err* with (q_err - [1;0;0;0])
%   and extracts the 3x1 vector part (elements 2:4) scaled by gain matrix K1.

% Inputs:
%   q_err - 4x1 attitude error quaternion [q0, q1, q2, q3]'
%   K1    - 3x3 kinematic control gain matrix (or k1 * I3)
%
% Output:
%   w_err_d - 3x1 virtual control angular velocity vector [w_err_d1; w_err_d2; w_err_d3] (rad/s)

    q_err = q_err(:);
    q_identity = [1; 0; 0; 0];

    % Quaternion difference: (q_err - [1; 0; 0; 0])
    q_diff = q_err - q_identity;

    % Conjugate of quaternion error: q_err*
    q_err_conj = quatConjugate(q_err);

    % Full 4D quaternion product: q_err* (x) (q_err - [1; 0; 0; 0])
    q_prod = quatMultiply(q_err_conj, q_diff);

    % Extract 3x1 vector part (elements 2:4) and multiply by gain K1
    w_err_d = -K1 * q_prod(2:4);
end
