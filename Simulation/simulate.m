function [t, state, quatErrHistory, wErrHistory, deltaHistory, torqueHistory, eulerRefHistory, wRefHistory] = simulate(params)
% SIMULATE Runs non-linear numerical ODE integration of quadrotor attitude backstepping control.
%
% Implements complete 13-step trajectory evaluation matching paper equations (Eq 8 to Eq 23):
%   Eq (8) : Kinematics dq/dt = 0.5 * q x [0; w]
%   Eq (9) : Dynamics dw/dt = J^-1 * (-w x J*w + tau)
%   Eq (10): Attitude error q_err = q_ref* x q
%   Eq (11): Angular velocity error w_err = w - w_ref
%   Eq (13): Virtual control w_err_d = -K1 * eps_err
%   Eq (15): Virtual control derivative d(w_err_d)/dt = -K1 * d(eps_err)/dt
%   Eq (16): Backstepping error delta = w_err - w_err_d
%   Eq (21): Feedback cancellation torque tau1 = w x J*w + J*dw_ref + J*dw_err_d - eps_err
%   Eq (22): Stabilizing control torque tau2 = -K2 * J * delta^(1/3)
%   Eq (23): Total torque tau = tau1 + tau2
%
% Output:
%   t               - Time vector (Nx1)
%   state           - System state history [q0..3, w1..3] (Nx7)
%   quatErrHistory  - Quaternion error history (Nx4)
%   wErrHistory     - Angular velocity error history (Nx3)
%   deltaHistory    - Backstepping error delta history (Nx3)
%   torqueHistory   - Total control torque history (Nx3)
%   eulerRefHistory - Reference Euler angles history (Nx3)
%   wRefHistory     - Reference angular velocity history (Nx3)

    % ODE options for fast, robust, high-precision integration
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-7);

    % Solve ODE differential equations using standard ode45
    [t, state] = ode45(@(t_curr, x_curr) droneODE(t_curr, x_curr, params), params.tSpan, params.x0, options);

    N = length(t);
    quatErrHistory  = zeros(N, 4);
    wErrHistory     = zeros(N, 3);
    deltaHistory    = zeros(N, 3);
    torqueHistory   = zeros(N, 3);
    eulerRefHistory = zeros(N, 3);
    wRefHistory     = zeros(N, 3);

    % Post-process history arrays for plot generation
    for i = 1:N
        t_curr = t(i);
        q = state(i, 1:4)';
        q = q / norm(q);
        w = state(i, 5:7)';

        % 1. Get reference trajectory values
        [q_ref, w_ref, dw_ref, euler_ref_deg] = params.getRefTrajectory(t_curr);

        % 2. Calculate controller states
        q_err    = quaternionError(q_ref, q);
        w_err    = angularVelocityError(w, w_ref);
        w_err_d  = desiredOmegaError(q_err, params.K1);
        delta    = deltaError(w_err, w_err_d);
        dw_err_d = desiredOmegaDerivative(q_err, w_err, w_ref, params.K1);

        % 3. Calculate torques (Eq 21, Eq 22, Eq 23)
        tau1 = torque1(w, params.J, dw_ref, dw_err_d, q_err);
        tau2 = torque2(delta, params.K2, params.J);
        tau  = totalTorque(tau1, tau2);

        % Store in history arrays
        quatErrHistory(i, :)  = q_err';
        wErrHistory(i, :)     = w_err';
        deltaHistory(i, :)    = delta';
        torqueHistory(i, :)   = tau';
        eulerRefHistory(i, :) = euler_ref_deg';
        wRefHistory(i, :)     = w_ref';
    end
end

%% Helper ODE Function for Dynamic State Derivatives
function dxdt = droneODE(t, x, params)
    q = x(1:4);
    q = q / norm(q);
    w = x(5:7);

    % 1. Get reference trajectory values
    [q_ref, w_ref, dw_ref, ~] = params.getRefTrajectory(t);

    % 2. Controller feedback calculation
    q_err    = quaternionError(q_ref, q);
    w_err    = angularVelocityError(w, w_ref);
    w_err_d  = desiredOmegaError(q_err, params.K1);
    delta    = deltaError(w_err, w_err_d);
    dw_err_d = desiredOmegaDerivative(q_err, w_err, w_ref, params.K1);

    % 3. Compute control torque tau (Eq 21 - 23)
    tau1 = torque1(w, params.J, dw_ref, dw_err_d, q_err);
    tau2 = torque2(delta, params.K2, params.J);
    tau  = totalTorque(tau1, tau2);

    % 4. Quadrotor dynamic equations
    dqdt = quaternionDynamics(q, w);
    dwdt = angularDynamics(w, params.J, tau);

    dxdt = [dqdt; dwdt];
end
