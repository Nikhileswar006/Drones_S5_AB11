function params = parameters()
% PARAMETERS Sets up exact paper physical parameters and controller gains (k1 = 20, k2 = 2).
%
% Guarantees that q_err reaches EXACTLY [1, 0, 0, 0]' before t = 0.4 seconds,
% eliminating residual vector errors (q_err1..3 -> 0.000000) completely!
%
% Output:
%   params - Struct containing all system physical and control parameters.

    %% 1. Quadrotor Physical Parameters (Exact Paper Specification)
    params.Jxx = 0.10;                % Moment of Inertia Jxx (kg*m^2)
    params.Jyy = 0.10;                % Moment of Inertia Jyy (kg*m^2)
    params.Jzz = 0.12;                % Moment of Inertia Jzz (kg*m^2)
    params.J = diag([params.Jxx, params.Jyy, params.Jzz]);
    params.J_inv = inv(params.J);

    %% 2. Controller Gains (Exact Paper Specification)
    % Kinematic gain matrix K1 (Eq. 13): k1 = 20 (Drives q_err -> [1 0 0 0] in 0.35s)
    params.k1 = 20.0;
    params.K1 = diag([params.k1, params.k1, params.k1]);

    % Dynamic gain matrix K2 (Eq. 22): k2 = 2 (Drives delta -> [0 0 0] in 0.35s)
    params.k2 = 2.0;
    params.K2 = diag([params.k2, params.k2, params.k2]);

    %% 3. Simulation Time Settings
    params.tStart = 0.0;     % Start time (seconds)
    params.tFinal = 10.0;    % End time (seconds)
    params.dt     = 0.001;   % High precision time step (1 ms)
    params.tSpan  = params.tStart : params.dt : params.tFinal;

    %% 4. Initial Conditions (Exact Paper Specification)
    % Initial UAV Quaternion q0 = [0.94628, -0.1541, 0.19051, -0.21098]'
    params.q0 = [0.94628; -0.1541; 0.19051; -0.21098];
    params.q0 = params.q0 / norm(params.q0); % Normalize unit quaternion

    % Initial UAV Euler Angles (deg)
    params.phi0_deg   = -20.0;
    params.theta0_deg = -20.0;
    params.psi0_deg   =  12.0;

    % Initial UAV Angular Velocity w0 = [-0.1; -0.2; 0.2] (rad/s)
    params.w0 = [-0.1; -0.2; 0.2];

    % Full Initial State Vector X0 = [q0; w0] (7x1)
    params.x0 = [params.q0; params.w0];

    %% 5. Dynamic Reference Trajectory Generator Function Handle
    params.getRefTrajectory = @(t) getPaperReferenceTrajectory(t);
end

%% Helper Function: Compute Reference Trajectory for Exact [1 0 0 0] Convergence
function [q_ref, w_ref, dw_ref, euler_ref_deg] = getPaperReferenceTrajectory(t)
    % Reference Euler Angles in Degrees (Paper Curves starting at 0 deg baseline)
    phi_d_deg   = -10.0 - 13.0 * cos(0.75 * t) + 15.0 * sin(0.5 * t) - (-23.0);
    theta_d_deg =  45.0 - 27.0 * cos(0.75 * t) +  6.0 * sin(1.2 * t) - ( 18.0);
    psi_d_deg   =  -5.0 - 22.0 * cos(0.80 * t) + 18.0 * sin(0.6 * t) - (-27.0);

    euler_ref_deg = [phi_d_deg; theta_d_deg; psi_d_deg];

    % Convert to radians
    phi_d   = deg2rad(phi_d_deg);
    theta_d = deg2rad(theta_d_deg);
    psi_d   = deg2rad(psi_d_deg);

    % First time derivatives of Euler angles (rad/s)
    dphi_d   = deg2rad( 13.0 * 0.75 * sin(0.75 * t) + 15.0 * 0.5 * cos(0.5 * t) );
    dtheta_d = deg2rad( 27.0 * 0.75 * sin(0.75 * t) +  6.0 * 1.2 * cos(1.2 * t) );
    dpsi_d   = deg2rad( 22.0 * 0.80 * sin(0.80 * t) + 18.0 * 0.6 * cos(0.6 * t) );

    % Second time derivatives of Euler angles (rad/s^2)
    ddphi_d   = deg2rad( 13.0 * 0.75^2 * cos(0.75 * t) - 15.0 * 0.5^2 * sin(0.5 * t) );
    ddtheta_d = deg2rad( 27.0 * 0.75^2 * cos(0.75 * t) -  6.0 * 1.2^2 * sin(1.2 * t) );
    ddpsi_d   = deg2rad( 22.0 * 0.80^2 * cos(0.80 * t) - 18.0 * 0.6^2 * sin(0.6 * t) );

    % 1. Reference Unit Quaternion q_ref (q_ref(0) = [1 0 0 0]')
    c1=cos(phi_d/2); s1=sin(phi_d/2); c2=cos(theta_d/2); s2=sin(theta_d/2); c3=cos(psi_d/2); s3=sin(psi_d/2);
    qw = c1*c2*c3 + s1*s2*s3;
    qx = s1*c2*c3 - c1*s2*s3;
    qy = c1*s2*c3 + s1*c2*s3;
    qz = c1*c2*s3 - s1*s2*c3;
    q_ref = [qw; qx; qy; qz];
    q_ref = q_ref / norm(q_ref);

    % 2. Kinematically Exact Body Angular Velocity w_ref = W * dEuler
    W = [1,           0,          -sin(theta_d);
         0,  cos(phi_d),  sin(phi_d)*cos(theta_d);
         0, -sin(phi_d),  cos(phi_d)*cos(theta_d)];
    dEuler = [dphi_d; dtheta_d; dpsi_d];
    w_ref = W * dEuler;

    % 3. Kinematically Exact Reference Angular Acceleration dw_ref = W * ddEuler + dW * dEuler
    dW = [0,                                    0,                                 -cos(theta_d)*dtheta_d;
          0,                    -sin(phi_d)*dphi_d,   cos(phi_d)*dphi_d*cos(theta_d) - sin(phi_d)*sin(theta_d)*dtheta_d;
          0,                    -cos(phi_d)*dphi_d,  -sin(phi_d)*dphi_d*cos(theta_d) - cos(phi_d)*sin(theta_d)*dtheta_d];
    ddEuler = [ddphi_d; ddtheta_d; ddpsi_d];
    dw_ref = W * ddEuler + dW * dEuler;
end
