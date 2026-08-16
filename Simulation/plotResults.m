function plotResults(t, state, quatErr, wErr, delta, torque, params, eulerRefHistory, wRefHistory)
% PLOTRESULTS Generates publication-quality figures with a crisp Dark Black background 
% for all figures (Fig 3 to Fig 12).
%
% Dark Theme Details:
%   - Outer Figure Background: Very Dark Gray / Black ([0.1 0.1 0.1])
%   - Plot Axes Background: Solid Black ('k')
%   - Grid Lines: Dark Gray ([0.35 0.35 0.35], Alpha = 0.6)
%   - Labels & Text: Light Gray / White ([0.85 0.85 0.85])
%   - Legend Box: Solid Black with White Text and Light Gray Border

    plotsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Plots');
    if ~exist(plotsDir, 'dir')
        mkdir(plotsDir);
    end

    % Set default graphics parameters
    set(0, 'DefaultAxesFontName', 'Times New Roman');
    set(0, 'DefaultAxesFontSize', 10);
    set(0, 'DefaultLineLineWidth', 1.8);

    N = length(t);
    euler_deg = zeros(N, 3);

    % Convert state quaternions to Euler angles (degrees)
    for i = 1:N
        [phi, theta, psi] = quaternionToEuler(state(i, 1:4)');
        euler_deg(i, :) = [rad2deg(phi), rad2deg(theta), rad2deg(psi)];
    end

    % Extract actual angular velocities
    w_actual = state(:, 5:7);
    idx1s = t <= 1.0; % Logical index for 0-1s transient window

    %% -------------------------------------------------------------------
    %% Master Composite Figure Page 1: Fig 3 to Fig 10 (Black Background)
    %% -------------------------------------------------------------------
    figMaster1 = figure('Name', 'IEEE/NTUST Paper Figures Page 1 (Fig 3 - Fig 10)', ...
                        'NumberTitle', 'off', 'Color', [0.1 0.1 0.1], 'Position', [30, 10, 1200, 950]);

    % Subplot 1: Fig. 3 Trajectory of quaternion error
    ax1 = subplot(4, 2, 1);
    plot(t(idx1s), quatErr(idx1s, 1), 'Color', [0.0 0.6 1.0], 'LineWidth', 2.2); hold on;
    plot(t(idx1s), quatErr(idx1s, 2), 'Color', [0.9 0.2 0.8], 'LineWidth', 1.8);
    plot(t(idx1s), quatErr(idx1s, 3), 'Color', [0.2 0.9 0.3], 'LineWidth', 1.8);
    plot(t(idx1s), quatErr(idx1s, 4), 'Color', [0.9 0.4 0.7], 'LineWidth', 1.8); grid on; hold off;
    ylim([-0.5, 1.1]); xlim([0, 1.0]);
    setDarkAxesStyle(ax1, 'Time (sec)', 'Quaternion Error', 'Fig. 3.  Trajectory of quaternion error');
    leg1 = legend(ax1, 'q_{err0}', 'q_{err1}', 'q_{err2}', 'q_{err3}', 'Location', 'east');
    setDarkLegendStyle(leg1);

    % Subplot 2: Fig. 7 Trajectory of angular velocity error
    ax2 = subplot(4, 2, 2);
    plot(t, wErr(:, 1), 'Color', [0.9 0.2 0.8], 'LineWidth', 1.8); hold on;
    plot(t, wErr(:, 2), 'Color', [0.9 0.3 0.3], 'LineWidth', 1.8);
    plot(t, wErr(:, 3), 'Color', [0.1 0.8 0.9], 'LineWidth', 1.8); grid on; hold off;
    setDarkAxesStyle(ax2, 'Time (seconds)', 'Angular velocity error(rad/sec)', 'Fig. 7.  Trajectory of angular velocity error');
    leg2 = legend(ax2, 'w_{err1}', 'w_{err2}', 'w_{err3}', 'Location', 'northeast');
    setDarkLegendStyle(leg2);

    % Subplot 3: Fig. 4 Trajectory of yaw angle tracking
    ax3 = subplot(4, 2, 3);
    plot(t, euler_deg(:, 3), 'Color', [0.9 0.1 0.6], 'LineWidth', 2.0); hold on;
    plot(t, eulerRefHistory(:, 3), 'Color', [0.95 0.75 0.1], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax3, 'Time (sec)', 'Yaw angle (degree)', 'Fig. 4.  Trajectory of yaw angle tracking');
    leg3 = legend(ax3, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg3);

    % Subplot 4: Fig. 8 Trajectory of roll angular velocity tracking
    ax4 = subplot(4, 2, 4);
    plot(t, w_actual(:, 1), 'Color', [0.9 0.4 0.2], 'LineWidth', 2.0); hold on;
    plot(t, wRefHistory(:, 1), 'Color', [0.9 0.2 0.8], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax4, 'Time (seconds)', 'Angular velocity(rad/sec)', 'Fig. 8.  Trajectory of roll angular velocity tracking');
    leg4 = legend(ax4, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg4);

    % Subplot 5: Fig. 5 Trajectory of pitch angle tracking
    ax5 = subplot(4, 2, 5);
    plot(t, euler_deg(:, 2), 'Color', [0.2 0.9 0.3], 'LineWidth', 2.0); hold on;
    plot(t, eulerRefHistory(:, 2), 'Color', [0.6 0.2 0.8], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax5, 'Time (sec)', 'Pitch angle(degree)', 'Fig. 5.  Trajectory of pitch angle tracking');
    leg5 = legend(ax5, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg5);

    % Subplot 6: Fig. 9 Trajectory of pitch angular velocity tracking
    ax6 = subplot(4, 2, 6);
    plot(t, w_actual(:, 2), 'Color', [0.1 0.5 0.9], 'LineWidth', 2.0); hold on;
    plot(t, wRefHistory(:, 2), 'Color', [0.3 0.6 0.9], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax6, 'Time (seconds)', 'Angular velocity(rad/sec)', 'Fig. 9.  Trajectory of pitch angular velocity tracking');
    leg6 = legend(ax6, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg6);

    % Subplot 7: Fig. 6 Trajectory of roll angle tracking
    ax7 = subplot(4, 2, 7);
    plot(t, euler_deg(:, 1), 'Color', [0.1 0.8 0.9], 'LineWidth', 2.0); hold on;
    plot(t, eulerRefHistory(:, 1), 'Color', [0.7 0.8 0.2], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax7, 'Time (sec)', 'Roll angle(degree)', 'Fig. 6.  Trajectory of roll angle tracking');
    leg7 = legend(ax7, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg7);

    % Subplot 8: Fig. 10 Trajectory of yaw angular velocity tracking
    ax8 = subplot(4, 2, 8);
    plot(t, w_actual(:, 3), 'Color', [0.9 0.4 0.2], 'LineWidth', 2.0); hold on;
    plot(t, wRefHistory(:, 3), 'Color', [0.7 0.3 0.3], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax8, 'Time (seconds)', 'Angular velocity(rad/sec)', 'Fig. 10.  Trajectory of yaw angular velocity tracking');
    leg8 = legend(ax8, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg8);

    saveas(figMaster1, fullfile(plotsDir, 'Paper_Figures_Page1_Fig3_to_Fig10.png'));

    %% -------------------------------------------------------------------
    %% Master Composite Figure Page 2: Fig 11 & Fig 12 (Black Background)
    %% -------------------------------------------------------------------
    figMaster2 = figure('Name', 'IEEE/NTUST Paper Figures Page 2 (Fig 11 - Fig 12)', ...
                        'NumberTitle', 'off', 'Color', [0.1 0.1 0.1], 'Position', [150, 50, 750, 850]);

    % Subplot 1: Fig. 11 Trajectory of \delta
    ax11 = subplot(2, 1, 1);
    plot(t, delta(:, 1), 'Color', [0.9 0.2 0.8], 'LineWidth', 1.8); hold on;
    plot(t, delta(:, 2), 'Color', [0.1 0.8 0.9], 'LineWidth', 1.8);
    plot(t, delta(:, 3), 'Color', [0.9 0.4 0.2], 'LineWidth', 1.8); grid on; hold off;
    ylim([-8.5, 4.5]); xlim([0, 10]);
    setDarkAxesStyle(ax11, 'Time (seconds)', '\delta(rad/sec)', 'Fig. 11.  Trajectory of \delta');
    leg11 = legend(ax11, '\delta_1', '\delta_2', '\delta_3', 'Location', 'east');
    setDarkLegendStyle(leg11);

    % Subplot 2: Fig. 12 Trajectory of torque (\tau)
    ax12 = subplot(2, 1, 2);
    plot(t, torque(:, 1), 'Color', [0.2 0.9 0.3], 'LineWidth', 1.8); hold on;
    plot(t, torque(:, 2), 'Color', [0.9 0.2 0.8], 'LineWidth', 1.8);
    plot(t, torque(:, 3), 'Color', [0.1 0.8 0.9], 'LineWidth', 1.8); grid on; hold off;
    ylim([-3.5, 4.5]); xlim([0, 10]);
    setDarkAxesStyle(ax12, 'Time (sec)', '\tau(N-m)', 'Fig. 12.  Trajectory of torque');
    leg12 = legend(ax12, '\tau_1', '\tau_2', '\tau_3', 'Location', 'northeast');
    setDarkLegendStyle(leg12);

    saveas(figMaster2, fullfile(plotsDir, 'Paper_Figures_Page2_Fig11_to_Fig12.png'));

    %% -------------------------------------------------------------------
    %% Individual High-Quality Figures (Black Background)
    %% -------------------------------------------------------------------
    % Fig 3
    f3 = figure('Name', 'Fig. 3 Trajectory of quaternion error', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f3);
    plot(t(idx1s), quatErr(idx1s, 1), 'Color', [0.0 0.6 1.0], 'LineWidth', 2.2); hold on;
    plot(t(idx1s), quatErr(idx1s, 2), 'Color', [0.9 0.2 0.8], 'LineWidth', 1.8);
    plot(t(idx1s), quatErr(idx1s, 3), 'Color', [0.2 0.9 0.3], 'LineWidth', 1.8);
    plot(t(idx1s), quatErr(idx1s, 4), 'Color', [0.9 0.4 0.7], 'LineWidth', 1.8); grid on; hold off;
    ylim([-0.5, 1.1]); xlim([0, 1.0]);
    setDarkAxesStyle(ax, 'Time (sec)', 'Quaternion Error', 'Fig. 3. Trajectory of quaternion error');
    leg = legend(ax, 'q_{err0}', 'q_{err1}', 'q_{err2}', 'q_{err3}', 'Location', 'east');
    setDarkLegendStyle(leg);
    saveas(f3, fullfile(plotsDir, 'Fig3_QuaternionError.png'));

    % Fig 4
    f4 = figure('Name', 'Fig. 4 Trajectory of yaw angle tracking', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f4);
    plot(t, euler_deg(:, 3), 'Color', [0.9 0.1 0.6], 'LineWidth', 2.0); hold on;
    plot(t, eulerRefHistory(:, 3), 'Color', [0.95 0.75 0.1], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax, 'Time (sec)', 'Yaw angle (degree)', 'Fig. 4. Trajectory of yaw angle tracking');
    leg = legend(ax, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg);
    saveas(f4, fullfile(plotsDir, 'Fig4_YawAngleTracking.png'));

    % Fig 5
    f5 = figure('Name', 'Fig. 5 Trajectory of pitch angle tracking', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f5);
    plot(t, euler_deg(:, 2), 'Color', [0.2 0.9 0.3], 'LineWidth', 2.0); hold on;
    plot(t, eulerRefHistory(:, 2), 'Color', [0.6 0.2 0.8], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax, 'Time (sec)', 'Pitch angle(degree)', 'Fig. 5. Trajectory of pitch angle tracking');
    leg = legend(ax, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg);
    saveas(f5, fullfile(plotsDir, 'Fig5_PitchAngleTracking.png'));

    % Fig 6
    f6 = figure('Name', 'Fig. 6 Trajectory of roll angle tracking', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f6);
    plot(t, euler_deg(:, 1), 'Color', [0.1 0.8 0.9], 'LineWidth', 2.0); hold on;
    plot(t, eulerRefHistory(:, 1), 'Color', [0.7 0.8 0.2], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax, 'Time (sec)', 'Roll angle(degree)', 'Fig. 6. Trajectory of roll angle tracking');
    leg = legend(ax, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg);
    saveas(f6, fullfile(plotsDir, 'Fig6_RollAngleTracking.png'));

    % Fig 7
    f7 = figure('Name', 'Fig. 7 Trajectory of angular velocity error', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f7);
    plot(t, wErr(:, 1), 'Color', [0.9 0.2 0.8], 'LineWidth', 1.8); hold on;
    plot(t, wErr(:, 2), 'Color', [0.9 0.3 0.3], 'LineWidth', 1.8);
    plot(t, wErr(:, 3), 'Color', [0.1 0.8 0.9], 'LineWidth', 1.8); grid on; hold off;
    setDarkAxesStyle(ax, 'Time (seconds)', 'Angular velocity error(rad/sec)', 'Fig. 7. Trajectory of angular velocity error');
    leg = legend(ax, 'w_{err1}', 'w_{err2}', 'w_{err3}', 'Location', 'northeast');
    setDarkLegendStyle(leg);
    saveas(f7, fullfile(plotsDir, 'Fig7_AngularVelocityError.png'));

    % Fig 8
    f8 = figure('Name', 'Fig. 8 Trajectory of roll angular velocity tracking', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f8);
    plot(t, w_actual(:, 1), 'Color', [0.9 0.4 0.2], 'LineWidth', 2.0); hold on;
    plot(t, wRefHistory(:, 1), 'Color', [0.9 0.2 0.8], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax, 'Time (seconds)', 'Angular velocity(rad/sec)', 'Fig. 8. Trajectory of roll angular velocity tracking');
    leg = legend(ax, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg);
    saveas(f8, fullfile(plotsDir, 'Fig8_RollAngularVelocityTracking.png'));

    % Fig 9
    f9 = figure('Name', 'Fig. 9 Trajectory of pitch angular velocity tracking', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f9);
    plot(t, w_actual(:, 2), 'Color', [0.1 0.5 0.9], 'LineWidth', 2.0); hold on;
    plot(t, wRefHistory(:, 2), 'Color', [0.3 0.6 0.9], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax, 'Time (seconds)', 'Angular velocity(rad/sec)', 'Fig. 9. Trajectory of pitch angular velocity tracking');
    leg = legend(ax, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg);
    saveas(f9, fullfile(plotsDir, 'Fig9_PitchAngularVelocityTracking.png'));

    % Fig 10
    f10 = figure('Name', 'Fig. 10 Trajectory of yaw angular velocity tracking', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f10);
    plot(t, w_actual(:, 3), 'Color', [0.9 0.4 0.2], 'LineWidth', 2.0); hold on;
    plot(t, wRefHistory(:, 3), 'Color', [0.7 0.3 0.3], 'LineWidth', 2.0); grid on; hold off;
    setDarkAxesStyle(ax, 'Time (seconds)', 'Angular velocity(rad/sec)', 'Fig. 10. Trajectory of yaw angular velocity tracking');
    leg = legend(ax, 'actual', 'reference', 'Location', 'northeast');
    setDarkLegendStyle(leg);
    saveas(f10, fullfile(plotsDir, 'Fig10_YawAngularVelocityTracking.png'));

    % Fig 11 (Trajectory of delta: Full 10s view)
    f11 = figure('Name', 'Fig. 11 Trajectory of delta', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f11);
    plot(t, delta(:, 1), 'Color', [0.9 0.2 0.8], 'LineWidth', 1.8); hold on;
    plot(t, delta(:, 2), 'Color', [0.1 0.8 0.9], 'LineWidth', 1.8);
    plot(t, delta(:, 3), 'Color', [0.9 0.4 0.2], 'LineWidth', 1.8); grid on; hold off;
    ylim([-8.5, 4.5]); xlim([0, 10]);
    setDarkAxesStyle(ax, 'Time (seconds)', '\delta(rad/sec)', 'Fig. 11. Trajectory of \delta');
    leg = legend(ax, '\delta_1', '\delta_2', '\delta_3', 'Location', 'east');
    setDarkLegendStyle(leg);
    saveas(f11, fullfile(plotsDir, 'Fig11_TrajectoryOfDelta.png'));

    % Fig 12 (Trajectory of torque: Full 10s view)
    f12 = figure('Name', 'Fig. 12 Trajectory of torque', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', f12);
    plot(t, torque(:, 1), 'Color', [0.2 0.9 0.3], 'LineWidth', 1.8); hold on;
    plot(t, torque(:, 2), 'Color', [0.9 0.2 0.8], 'LineWidth', 1.8);
    plot(t, torque(:, 3), 'Color', [0.1 0.8 0.9], 'LineWidth', 1.8); grid on; hold off;
    ylim([-3.5, 4.5]); xlim([0, 10]);
    setDarkAxesStyle(ax, 'Time (sec)', '\tau(N-m)', 'Fig. 12. Trajectory of torque');
    leg = legend(ax, '\tau_1', '\tau_2', '\tau_3', 'Location', 'northeast');
    setDarkLegendStyle(leg);
    saveas(f12, fullfile(plotsDir, 'Fig12_TrajectoryOfTorque.png'));

    fprintf('   - All Paper Figures generated with crisp Dark Black Background and saved to: %s\n', plotsDir);
end

%% Helper Function to Apply Dark Theme Axes Styling
function setDarkAxesStyle(ax, xLabelText, yLabelText, titleText)
    set(ax, 'Color', 'k', 'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8], ...
            'GridColor', [0.35 0.35 0.35], 'GridAlpha', 0.6, 'Box', 'on');
    xlabel(ax, xLabelText, 'Color', [0.85 0.85 0.85], 'FontSize', 10);
    ylabel(ax, yLabelText, 'Color', [0.85 0.85 0.85], 'FontSize', 10);
    title(ax, titleText, 'Color', [0.85 0.85 0.85], 'FontSize', 10, 'FontWeight', 'normal');
end

%% Helper Function to Apply Dark Theme Legend Styling
function setDarkLegendStyle(leg)
    if ~isempty(leg)
        set(leg, 'Color', 'k', 'TextColor', 'w', 'EdgeColor', [0.5 0.5 0.5]);
    end
end
