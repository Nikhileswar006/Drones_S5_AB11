function tau1 = torque1(w, J, dw_ref, dw_err_d, ~)
% TORQUE1 Computes non-linear feedback cancellation torque tau1.
%
% Implements exact Paper Equation (21):
%   tau1 = w x (J * w) + J * dw_ref + J * dw_err_d
%
% Inputs:
%   w        - 3x1 body angular velocity [wx, wy, wz]' (rad/s)
%   J        - 3x3 inertia matrix (kg*m^2)
%   dw_ref   - 3x1 reference angular acceleration [dw_ref1, dw_ref2, dw_ref3]' (rad/s^2)
%   dw_err_d - 3x1 virtual control derivative [dw_err_d1, dw_err_d2, dw_err_d3]' (rad/s^2)
%
% Output:
%   tau1     - 3x1 feedback cancellation torque vector [tau1_x; tau1_y; tau1_z] (N*m)

    w = w(:);
    dw_ref = dw_ref(:);
    dw_err_d = dw_err_d(:);

    % Gyroscopic cross product term: w x (J * w)
    gyroscopic = cross(w, J * w);

    % Exact Paper Equation (21): tau1 = w x (J * w) + J * dw_ref + J * dw_err_d
    tau1 = gyroscopic + J * dw_ref + J * dw_err_d;
end
