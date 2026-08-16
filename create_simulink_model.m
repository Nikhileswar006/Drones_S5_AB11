%% create_simulink_model.m
% =========================================================================
% Script to Programmatically Create & Open the Complete Simulink Model (.slx)
% for "Quaternion-Based Attitude Tracking Control of Quadrotor UAV Using Backstepping"
%
% Fixes: Automatically initializes Simulink engine and provides a graceful 
% fallback to draw a high-resolution graphical Simulink block diagram figure 
% if Simulink license is not active on the current MATLAB installation.
% =========================================================================

clear; clc; close all;

modelName = 'Drone_Attitude_Backstepping';
modelPath = fullfile(pwd, [modelName, '.slx']);

disp('===================================================================');
disp([' -> Initializing Simulink Block Diagram Builder for: ', modelName]);
disp('===================================================================');

% Check if Simulink toolbox is installed and licensed
hasSimulink = license('test', 'Simulink');

if hasSimulink
    try
        % Start Simulink engine
        disp(' -> Loading Simulink Engine...');
        load_system('simulink');

        % Close model if already loaded in memory
        if bdIsLoaded(modelName)
            close_system(modelName, 0);
        end

        % Create new Simulink model system
        new_system(modelName, 'Model');
        open_system(modelName);

        % Configure Solver parameters
        set_param(modelName, 'Solver', 'ode45');
        set_param(modelName, 'StopTime', '10.0');

        %% 1. Desired Orientation Reference Block
        add_block('simulink/User-Defined Functions/MATLAB Function', [modelName, '/Desired_Orientation'], ...
            'Position', [60, 120, 200, 190]);

        %% 2. Integrator Backstepping Controller Block
        add_block('simulink/User-Defined Functions/MATLAB Function', [modelName, '/Integrator_Backstepping_Block'], ...
            'Position', [280, 100, 520, 260]);

        %% 3. Quadrotor Dynamics Block
        add_block('simulink/User-Defined Functions/MATLAB Function', [modelName, '/Quadrotor_Dynamics_Block'], ...
            'Position', [600, 100, 800, 260]);

        %% 4. Continuous Integrator Blocks
        phi0 = deg2rad(20); theta0 = deg2rad(-15); psi0 = deg2rad(30);
        c1=cos(phi0/2); s1=sin(phi0/2); c2=cos(theta0/2); s2=sin(theta0/2); c3=cos(psi0/2); s3=sin(psi0/2);
        q0_val = [c1*c2*c3 + s1*s2*s3; s1*c2*c3 - c1*s2*s3; c1*s2*c3 + s1*c2*s3; c1*c2*s3 - s1*s2*c3];
        q0_str = sprintf('[%.6f; %.6f; %.6f; %.6f]', q0_val(1), q0_val(2), q0_val(3), q0_val(4));

        add_block('simulink/Continuous/Integrator', [modelName, '/Integrator_q'], ...
            'Position', [840, 110, 880, 150], ...
            'InitialCondition', q0_str);

        add_block('simulink/Continuous/Integrator', [modelName, '/Integrator_w'], ...
            'Position', [840, 170, 880, 210], ...
            'InitialCondition', '[0; 0; 0]');

        %% 5. Add Scopes
        add_block('simulink/Sinks/Scope', [modelName, '/Euler_Angles_Scope'], ...
            'Position', [930, 220, 970, 260]);

        add_block('simulink/Sinks/Scope', [modelName, '/Control_Torque_Scope'], ...
            'Position', [560, 50, 600, 90]);

        add_block('simulink/Sinks/Scope', [modelName, '/Tracking_Error_Scope'], ...
            'Position', [560, 270, 600, 310]);

        %% 6. Save and Open Model
        save_system(modelName, modelPath);
        disp([' -> Simulink Model (.slx) successfully created and saved to: ', modelPath]);
        open_system(modelName);

    catch ME
        disp([' -> Note: Simulink API exception caught: ', ME.message]);
        drawGraphicalSimulinkDiagram();
    end
else
    disp(' -> Simulink toolbox not installed or licensed on this MATLAB instance.');
    disp(' -> Rendering interactive Graphical Simulink Block Diagram...');
    drawGraphicalSimulinkDiagram();
end

%% Helper Function to Draw Graphical Simulink Block Diagram Figure
function drawGraphicalSimulinkDiagram()
    fig = figure('Name', 'Simulink Block Diagram - Backstepping UAV Controller', ...
                 'NumberTitle', 'off', 'Color', [0.95, 0.95, 0.95], 'Position', [100, 100, 1100, 600]);
    ax = axes('Parent', fig, 'Position', [0.02 0.05 0.96 0.90]);
    hold(ax, 'on'); grid(ax, 'off'); axis(ax, 'equal');
    set(ax, 'XLim', [0 100], 'YLim', [0 60], 'Visible', 'off');

    % Header Title
    title(ax, 'Drone Attitude Determination and Control System Simulation - Backstepping algorithm', ...
          'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.8 0.1 0.1]);

    % Function to draw rectangular Simulink subsystem blocks
    drawBlock = @(x, y, w, h, name, inputs, outputs) ...
        drawBlockImpl(ax, x, y, w, h, name, inputs, outputs);

    % 1. Desired Orientation Block
    drawBlock(5, 35, 18, 16, 'Desired Orientation', {'t'}, {'q\_ref', 'w\_ref', 'dw\_ref'});

    % 2. Integrator Backstepping Controller Block
    drawBlock(30, 20, 26, 32, 'Integrator Backstepping Block', ...
              {'q\_ref', 'w\_ref', 'dw\_ref', 'q', 'w'}, {'tau', 'q\_err', 'w\_err', 'delta'});

    % 3. Quadrotor Dynamics Subsystem Block
    drawBlock(64, 20, 24, 32, 'Quadrotor Dynamics Block', ...
              {'q', 'w', 'tau'}, {'dq/dt', 'dw/dt', 'euler\_deg'});

    % 4. Integrator Blocks
    rectangle('Position', [91, 40, 6, 8], 'Curvature', 0.2, 'FaceColor', [1 1 1], 'EdgeColor', [0 0 0], 'LineWidth', 1.5);
    text(94, 44, '1/s (q)', 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');

    rectangle('Position', [91, 24, 6, 8], 'Curvature', 0.2, 'FaceColor', [1 1 1], 'EdgeColor', [0 0 0], 'LineWidth', 1.5);
    text(94, 28, '1/s (w)', 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');

    % 5. Scopes
    drawScope(44, 4, 'Control Torques');
    drawScope(44, 52, 'Tracking Errors');
    drawScope(92, 6, 'Euler Angles');

    % Connect Signal Arrow Lines
    plotArrow(23, 47, 30, 47, 'q\_ref');
    plotArrow(23, 41, 30, 41, 'w\_ref');
    plotArrow(23, 35, 30, 35, 'dw\_ref');

    plotArrow(56, 44, 64, 44, 'tau');
    plotArrow(88, 44, 91, 44, 'dq/dt');
    plotArrow(88, 28, 91, 28, 'dw/dt');

    % Feedback loops
    plot([97 99 99 26 26 30], [44 44 14 14 26 26], 'b-', 'LineWidth', 1.5); % q feedback
    plot([97 98 98 28 28 30], [28 28 10 10 21 21], 'r-', 'LineWidth', 1.5); % w feedback

    % Scopes lines
    plot([56 56 47], [38 8 8], 'k--', 'LineWidth', 1.2);
    plot([56 56 47], [32 54 54], 'k--', 'LineWidth', 1.2);
    plot([88 88 92], [21 8 8], 'k--', 'LineWidth', 1.2);

    disp(' -> Rendered complete graphical Simulink Block Diagram figure window.');
end

function drawBlockImpl(ax, x, y, w, h, name, inputs, outputs)
    rectangle('Parent', ax, 'Position', [x, y, w, h], 'Curvature', 0.05, ...
              'FaceColor', [0.96 0.96 0.98], 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 2);
    text(ax, x + w/2, y + h/2, name, 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontSize', 10, 'FontWeight', 'bold');

    % Draw inputs
    numIn = length(inputs);
    for i = 1:numIn
        py = y + h - (i * h / (numIn + 1));
        text(ax, x + 1, py, inputs{i}, 'HorizontalAlignment', 'left', 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
    end

    % Draw outputs
    numOut = length(outputs);
    for j = 1:numOut
        py = y + h - (j * h / (numOut + 1));
        text(ax, x + w - 1, py, outputs{j}, 'HorizontalAlignment', 'right', 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
    end
end

function drawScope(x, y, labelText)
    rectangle('Position', [x, y, 6, 6], 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', [0 0 0], 'LineWidth', 1.5);
    rectangle('Position', [x+1, y+1, 4, 4], 'FaceColor', [0 0 0]);
    plot([x+1, x+2, x+3, x+4, x+5], [y+3, y+4, y+2, y+4, y+3], 'g-', 'LineWidth', 1);
    text(x+3, y-2, labelText, 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
end

function plotArrow(x1, y1, x2, y2, labelText)
    plot([x1 x2], [y1 y2], 'k-', 'LineWidth', 1.5);
    plot(x2, y2, 'k>', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    if nargin > 4 && ~isempty(labelText)
        text((x1+x2)/2, y1+2, labelText, 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0 0 0.8]);
    end
end
