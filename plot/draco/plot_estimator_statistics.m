close all;
clear;
clc;

addpath("/tmp")
addpath("experiment_data")
addpath("plot")

exp_data_location = 'experiment_data';
base_estimator_type = {'est', 'kf'};    % 'est' = kinematics-only
est_or_kf = 2;                        % 1: est, 2: kf

dd = dir(sprintf("%s/draco_controller_data*.mat", exp_data_location));
ddd = dir(sprintf("%s/draco_state_estimator_kf_*.mat", exp_data_location));


[~, i] = max([dd.datenum]);
fprintf('loading %s \n', dd(i).name)
load(dd(i).name, 'time')
load(dd(i).name, 'state')
load(dd(i).name, 'des_lf_pos')
load(dd(i).name, 'des_rf_pos')
load(dd(i).name, 'act_lf_pos')
load(dd(i).name, 'act_rf_pos')
load(dd(i).name, 'des_lf_vel')
load(dd(i).name, 'des_rf_vel')
load(dd(i).name, 'act_lf_vel')
load(dd(i).name, 'act_rf_vel')
load(dd(i).name, 'local_des_lf_pos')
load(dd(i).name, 'local_des_rf_pos')
load(dd(i).name, 'local_act_lf_pos')
load(dd(i).name, 'local_act_rf_pos')
load(dd(i).name, 'local_des_lf_vel')
load(dd(i).name, 'local_des_rf_vel')
load(dd(i).name, 'local_act_lf_vel')
load(dd(i).name, 'local_act_rf_vel')

[~, i] = max([ddd.datenum]);
fprintf('loading %s \n', ddd(i).name)
load(ddd(i).name)

load_draco_label_names
load_colors

%source functions
mfilepath = fileparts(which('plot_task'));
addpath(fullfile(erase(mfilepath, '/draco')));
helper_functions

num_wbc_data = length(icp_est);
wbc_time = time(end-num_wbc_data+1:end);
num_fig = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%left foot pos task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, local_des_lf_pos(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_act_lf_pos(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_lf_pos(j,:), local_act_lf_pos(j,:)]);
        max_val = max([local_des_lf_pos(j,:), local_act_lf_pos(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_label(j))
    else
        k = k + 1;
        plot(wbc_time, local_des_lf_vel(k, :), 'r', 'LineWidth', 3);
         hold on
        plot(wbc_time, local_act_lf_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_lf_vel(j,:), local_act_lf_vel(j,:)]);
        max_val = max([local_des_lf_vel(j,:), local_act_lf_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Left Foot Task (Local)', 'FontSize', 30)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%right foot pos task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, des_rf_pos(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_rf_pos(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_rf_pos(j,:), act_rf_pos(j,:)]);
        max_val = max([des_rf_pos(j,:), act_rf_pos(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_label(j))
    else
        k = k + 1;
        plot(wbc_time, des_rf_vel(k, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_rf_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_rf_vel(j,:), act_rf_vel(j,:)]);
        max_val = max([des_rf_vel(j,:), act_rf_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Right Foot Task (Local)', 'FontSize', 30)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%IMU accelerometer (using dVel/dt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    ax(i) = subplot(3,1,i);
    plot(wbc_time, imu_accel_raw(i,:), 'r', 'LineWidth', 3)
    hold on
    plot(wbc_time, imu_accel_est(i,:), 'b', 'LineWidth', 2)
    grid on
    min_val = min([imu_accel_raw(j,:), imu_accel_est(j,:)]);
    max_val = max([imu_accel_raw(j,:), imu_accel_est(j,:)]);
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(xyz_label(i))
end
sgtitle('IMU accel in Balance State', 'FontSize', 30)
linkaxes(ax)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% KF histograms
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
discount_init = 80;
discount_end = 100;
balance_state = 3;
wbc_state = state(end-num_wbc_data+discount_init:end-discount_end);
bal_rf_pos = local_act_rf_pos(:, wbc_state == balance_state);
bal_rf_vel = local_act_rf_vel(:, wbc_state == balance_state);
bal_lf_pos = local_act_lf_pos(:, wbc_state == balance_state);
bal_lf_vel = local_act_lf_vel(:, wbc_state == balance_state);

% compute feet position errors
bal_lf_pos_mu = mean(bal_lf_pos, 2);
bal_rf_pos_mu = mean(bal_rf_pos, 2);
bal_lf_pos_error = bal_lf_pos - bal_lf_pos_mu;
bal_rf_pos_error = bal_rf_pos - bal_rf_pos_mu;

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        histfit(bal_lf_pos_error(j, :))
        pd = fitdist(bal_lf_pos_error(j,:)', 'Normal');
        mu_fit_str = sprintf('mu = %.4f', pd.mu);
        sigma_fit_str = sprintf('sigma = %.4f', pd.sigma);
        annotation('textbox', [0.35, 0.8-0.3*(j-1), 0.1, 0.1], 'String', mu_fit_str)
        annotation('textbox', [0.35, 0.76-0.3*(j-1), 0.1, 0.1], 'String', sigma_fit_str)
        xlabel('position error (m)')
        ylabel(xyz_label(j))
    else
        k = k + 1;
        histfit(bal_lf_vel(k, :))
        pd = fitdist(bal_lf_vel(k,:)', 'Normal');
        mu_fit_str = sprintf('mu = %.4f', pd.mu);
        sigma_fit_str = sprintf('sigma = %.4f', pd.sigma);
        annotation('textbox', [0.8, 0.8-0.3*(k-1), 0.1, 0.1], 'String', mu_fit_str)
        annotation('textbox', [0.8, 0.76-0.3*(k-1), 0.1, 0.1], 'String', sigma_fit_str)
        xlabel('velocity (m/s)')
        ylabel(xyz_label(k))
    end
    sgtitle('Left Foot in Balance State', 'FontSize', 30)
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        histfit(bal_rf_pos_error(j, :))
        pd = fitdist(bal_rf_pos_error(j,:)', 'Normal');
        mu_fit_str = sprintf('mu = %.4f', pd.mu);
        sigma_fit_str = sprintf('sigma = %.4f', pd.sigma);
        annotation('textbox', [0.35, 0.8-0.3*(j-1), 0.1, 0.1], 'String', mu_fit_str)
        annotation('textbox', [0.35, 0.76-0.3*(j-1), 0.1, 0.1], 'String', sigma_fit_str)
        xlabel('position error (m)')
        ylabel(xyz_label(j))
    else
        k = k + 1;
        histfit(bal_rf_vel(k, :))
        pd = fitdist(bal_rf_vel(k,:)', 'Normal');
        mu_fit_str = sprintf('mu = %.4f', pd.mu);
        sigma_fit_str = sprintf('sigma = %.4f', pd.sigma);
        annotation('textbox', [0.8, 0.8-0.3*(k-1), 0.1, 0.1], 'String', mu_fit_str)
        annotation('textbox', [0.8, 0.76-0.3*(k-1), 0.1, 0.1], 'String', sigma_fit_str)
        xlabel('velocity (m/s)')
        ylabel(xyz_label(k))
    end
    sgtitle('Right Foot in Balance State', 'FontSize', 30)
end

% compute imu accel errors
bal_accel = imu_accel_est(:, wbc_state == balance_state);
bal_accel_mu = mean(bal_accel, 2);
bal_accel_errors = bal_accel - bal_accel_mu;

% Raw values
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    ax(i) = subplot(3,1,i);
    histfit(bal_accel(i, :))
    pd = fitdist(bal_accel(i,:)', 'Normal');
    mu_fit_str = sprintf('mu = %.4f', pd.mu);
    sigma_fit_str = sprintf('sigma = %.4f', pd.sigma);
    annotation('textbox', [0.75, 0.8-0.3*(i-1), 0.1, 0.1], 'String', mu_fit_str)
    annotation('textbox', [0.75, 0.76-0.3*(i-1), 0.1, 0.1], 'String', sigma_fit_str)
    xlabel('imu accel (m/s^2)')
    ylabel(xyz_label(i))
end
sgtitle('IMU accel in Balance State', 'FontSize', 30)

% Errors
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    ax(i) = subplot(3,1,i);
    histfit(bal_accel_errors(i, :))
    pd = fitdist(bal_accel_errors(i,:)', 'Normal');
    mu_fit_str = sprintf('mu = %.4f', pd.mu);
    sigma_fit_str = sprintf('sigma = %.4f', pd.sigma);
    annotation('textbox', [0.75, 0.8-0.3*(i-1), 0.1, 0.1], 'String', mu_fit_str)
    annotation('textbox', [0.75, 0.76-0.3*(i-1), 0.1, 0.1], 'String', sigma_fit_str)
    xlabel('imu accel error (m/s^2)')
    ylabel(xyz_label(i))
end
sgtitle('IMU accel error in Balance State', 'FontSize', 30)

