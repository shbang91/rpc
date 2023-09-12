close all;
clear;
clc;

addpath("/tmp")
addpath("experiment_data")
addpath("plot")

exp_data_location = '/tmp';
base_estimator_type = {'est', 'kf'};    % 'est' = kinematics-only
est_or_kf = 1;                        % 1: est, 2: kf

d = dir(sprintf("%s/draco_icp_data*.mat", exp_data_location));
dd = dir(sprintf("%s/draco_controller_data*.mat", exp_data_location));
% ddd = dir(sprintf("%s/draco_state_estimator_kf_*.mat", exp_data_location));
ddd = dir("/tmp/draco_state_estimator_data*.mat");

[tmp, i] = max([d.datenum]);
fprintf('loading %s \n', d(i).name)
load(d(i).name, "icp_error_raw")
load(d(i).name, "icp_avg_err")


[tmp, i] = max([dd.datenum]);
fprintf('loading %s \n', dd(i).name)
load(dd(i).name, 'time')
load(dd(i).name, 'state')

[tmp, i] = max([ddd.datenum]);
fprintf('loading %s \n', ddd(i).name)
load(ddd(i).name)

load_draco_label_names
load_colors

%source functions
mfilepath = fileparts(which('plot_state_estimator'));
addpath(fullfile(erase(mfilepath, '/draco')));
helper_functions

%%
num_wbc_data = length(com_vel_raw);
wbc_time = time(end-num_wbc_data+1:end);

%%
num_fig = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%floating base estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    subplot(3,1, i);
    pos_est = eval(sprintf('base_joint_pos_%s', base_estimator_type{est_or_kf}));
    plot(wbc_time, pos_est(i, :), 'b', 'LineWidth',2);
    grid on
    hold on
    min_val = min(pos_est(i,:));
    max_val = max(pos_est(i,:));
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(xyz_label(i))
    sgtitle(sprintf('base joint pos %s', base_estimator_type{est_or_kf}), 'FontSize', 30)
end

figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    subplot(3, 1, i);
    rpy_est = eval(sprintf('base_joint_rpy_%s', base_estimator_type{est_or_kf}));
    plot(wbc_time, rpy_est(i, :), 'b', 'LineWidth',2);
    grid on
    hold on
    min_val = min(rpy_est(i,:));
    max_val = max(rpy_est(i,:));
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(rpy_label(i))
    sgtitle(sprintf('base joint rpy %s', base_estimator_type{est_or_kf}), 'FontSize', 30)
end

% figure(num_fig)
% num_fig = num_fig + 1;
% j=0;
% for i = 3:-1:1
%     j = j+1;
%     subplot(3, 1, j);
%     plot(wbc_time, base_joint_ypr_raw(i, :), 'b', 'LineWidth',2);
%     grid on
%     hold on
%     min_val = min(base_joint_ypr_raw(i,:));
%     max_val = max(base_joint_ypr_raw(i,:));
%     min_val = min_val - 0.1 * (max_val - min_val);
%     max_val = max_val + 0.1 *(max_val - min_val);
%     set_fig_opt()
%     plot_phase(time, state, min_val, max_val, phase_color)
%     xlabel('time')
%     ylabel(rpy_label(j))
%     sgtitle('base joint rpy Raw (IMU Frame Quat)', 'FontSize', 30)
% end


% figure(num_fig)
% num_fig = num_fig + 1;
% for i = 1:3
%     subplot(3, 1, i);
%     plot(wbc_time, base_joint_rpy_error(i, :), 'b', 'LineWidth',2);
%     grid on
%     hold on
%     min_val = min(base_joint_rpy_error(i,:));
%     max_val = max(base_joint_rpy_error(i,:));
%     min_val = min_val - 0.1 * (max_val - min_val);
%     max_val = max_val + 0.1 *(max_val - min_val);
%     set_fig_opt()
%     plot_phase(time, state, min_val, max_val, phase_color)
%     xlabel('time')
%     ylabel(rpy_label(i))
%     sgtitle('base joint rpy Error', 'FontSize', 30)
% end

figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    ax(i) = subplot(3, 1, i);
    lin_vel_est = eval(sprintf('base_joint_lin_vel_%s', base_estimator_type{est_or_kf}));
    plot(wbc_time, lin_vel_est(i, :), 'b', 'LineWidth',2);
    grid on
    hold on
    min_val = min(lin_vel_est(i,:));
    max_val = max(lin_vel_est(i,:));
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(xyz_label(i))
    sgtitle(sprintf('base joint lin vel %s', base_estimator_type{est_or_kf}), 'FontSize', 30)
end 
linkaxes(ax, 'x')

figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    subplot(3, 1, i);
    ang_vel_est = eval(sprintf('base_joint_ang_vel_%s', base_estimator_type{est_or_kf}));
    plot(wbc_time, ang_vel_est(i, :), 'b', 'LineWidth',2);
    grid on
    hold on
    min_val = min(ang_vel_est(i,:));
    max_val = max(ang_vel_est(i,:));
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(ang_vel_label(i))
    sgtitle('base joint ang vel est', 'FontSize', 30)
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% com vel est
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
     ax(i) = subplot(3,1,i);
        plot(wbc_time, com_vel_raw(i, :), 'k', 'LineWidth', 3);
        hold on
        plot(wbc_time, com_vel_est(i, :), 'r', 'LineWidth', 2);
        grid on
        min_val = min([com_vel_raw(i,:), com_vel_est(i,:)]);
        max_val = max([com_vel_raw(i,:), com_vel_est(i,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(i))
        legend('raw', 'est')
    sgtitle('com vel est', 'FontSize', 30)
end
linkaxes(ax, 'x')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% icp est
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:4
    ax(i) = subplot(2, 2, i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, icp_est(j, :), 'b', 'LineWidth',2);
        grid on
        hold on
        min_val = min(icp_est(j, :));
        max_val = max(icp_est(j, :));
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xy_label(j))
        if j == 1
            title('icp est', 'FontSize', 30)
        end
    else
        k = k + 1;
        plot(wbc_time, icp_vel_est(k, :), 'b', 'LineWidth', 2);
        grid on
        hold on
        min_val = min(icp_vel_est(k, :));
        max_val = max(icp_vel_est(k, :));
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xy_dot_label(k))
        if k == 1
            title('icp vel est', 'FontSize', 30)
        end
    end
end
linkaxes(ax, 'x')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%icp err plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:2
    ax(i) = subplot(2,1,i);
        plot(wbc_time, icp_error_raw(i, :), 'k', 'LineWidth', 3);
        hold on
        plot(wbc_time, icp_avg_err(i, :), 'r', 'LineWidth', 2);
        grid on
        min_val = min([icp_error_raw(i,:), icp_avg_err(i,:)]);
        max_val = max([icp_error_raw(i,:), icp_avg_err(i,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xy_label(i))
    sgtitle('icp integrator', 'FontSize', 30)
end
linkaxes(ax,'x')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%imu accel plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    ax(i) = subplot(3,1,i);
    plot(wbc_time, imu_accel_raw(i, :), 'k', 'LineWidth', 3);
    hold on
    plot(wbc_time, imu_accel_est(i, :), 'r', 'LineWidth', 2);
    grid on
    min_val = min([imu_accel_raw(i,:), imu_accel_est(i,:)]);
    max_val = max([imu_accel_raw(i,:), imu_accel_est(i,:)]);
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(xyz_label(i))
end
sgtitle('IMU Lin Accel', 'FontSize', 30)
linkaxes(ax,'x')