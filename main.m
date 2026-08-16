%% main.m
% =========================================================================
% IEEE / NTUST Paper Title:
% "Quaternion-Based Attitude Tracking Control Design for UAVs"
%
% Description:
% Master execution script implementing EXACT published paper parameters
% and Equation (22) finite-time fractional power term: tau2 = -k2 * J * delta^(1/3).
% =========================================================================

clear; clc; close all;

%% 1. Path Configuration
projectDir = fileparts(mfilename('fullpath'));
addpath(fullfile(projectDir, 'Quaternion'));
addpath(fullfile(projectDir, 'Controller'));
addpath(fullfile(projectDir, 'Dynamics'));
addpath(fullfile(projectDir, 'Simulation'));
addpath(fullfile(projectDir, 'Utils'));

disp('===================================================================');
disp('   QUATERNION-BASED BACKSTEPPING ATTITUDE CONTROL OF QUADROTOR UAV');
disp('   Reproduction with Exact Published Paper Parameters & Eq 22 (tau2)');
disp('===================================================================');

%% 2. Load System Parameters & Initial Conditions
disp('-> Loading exact paper physical parameters and controller gains (parameters.m)...');
params = parameters();

fprintf('   - Inertia J: diag([%.2f, %.2f, %.2f]) kg*m^2\n', params.Jxx, params.Jyy, params.Jzz);
fprintf('   - Controller Gain k1: %.1f\n', params.k1);
fprintf('   - Controller Gain k2: %.1f\n', params.k2);
fprintf('   - Initial Quaternion q0: [%.5f, %.5f, %.5f, %.5f]''\n', ...
        params.q0(1), params.q0(2), params.q0(3), params.q0(4));
fprintf('   - Initial Angular Velocity w0: [%.2f, %.2f, %.2f]'' rad/s\n', ...
        params.w0(1), params.w0(2), params.w0(3));

%% 3. Run Simulation
disp('-> Launching non-linear numerical ODE simulation...');
tic;
[t, state, quatErr, wErr, delta, torque, eulerRefHistory, wRefHistory] = simulate(params);
simTime = toc;
fprintf('-> Simulation completed in %.4f seconds.\n', simTime);

%% 4. Plot Figures Matching Paper Layout (Fig 3 - Fig 12)
disp('-> Generating exact dark-theme paper figures (Fig. 3 - Fig. 12)...');
plotResults(t, state, quatErr, wErr, delta, torque, params, eulerRefHistory, wRefHistory);

%% 5. Display Summary Statistics
q_final_err = quatErr(end, :);
w_final_err = wErr(end, :);

disp('===================================================================');
disp('                     SIMULATION SUMMARY RESULTS                    ');
disp('===================================================================');
fprintf(' Final Quaternion Vector Error norm ||q_err(1:3)|| : %.6e\n', norm(q_final_err(2:4)));
fprintf(' Final Angular Velocity Error norm ||w_err||       : %.6e rad/s\n', norm(w_final_err));
fprintf(' Peak Control Torque ||tau_max||                  : %.4f N*m\n', max(sqrt(sum(torque.^2, 2))));
disp('===================================================================');
disp(' Simulation executed successfully. All figures generated.');
