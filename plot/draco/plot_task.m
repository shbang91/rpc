close all;
clear;
clc;

addpath("/tmp")
addpath("experiment_data")
% addpath("plot")
addpath("plot/draco")

% exp_data_location = 'experiment_data';
exp_data_location = '/tmp';

b_include_lhand_task = false;

d = dir(sprintf("%s/draco_controller_data*.mat", exp_data_location));
[tmp, i] = max([d.datenum]);
fprintf('loading %s \n', d(i).name)
load(d(i).name)

dd = dir(sprintf("%s/draco_state_estimator_data*.mat", exp_data_location));
% dd = dir(sprintf("%s/draco_state_estimator_kf_data*.mat", exp_data_location));
[tmp, i] = max([dd.datenum]);
fprintf('loading %s \n', dd(i).name)
load(dd(i).name, 'joint_pos_act')
load(dd(i).name, 'joint_vel_act')
load(dd(i).name, 'icp_est')

ddd = dir(sprintf("%s/draco_icp_data*.mat", exp_data_location));
[tmp, i] = max([ddd.datenum]);
fprintf('loading %s \n', ddd(i).name)
load(ddd(i).name, 'des_icp')
% load(ddd(i).name, 'act_icp')
load(ddd(i).name, 'local_des_icp')
load(ddd(i).name, 'local_act_icp')

%%
load_draco_label_names
load_colors
task_names = {'com_xy_task', 'com_z_task', 'joint_task', ...
    'lf_force_task', 'lf_ori_task', 'lf_pos_task', ...
    'rf_force_task', 'rf_ori_task', 'rf_pos_task', ...
    'qddot_regularization_task', 'Fr_regularization_task', 'torso_ori_task', 'upper_body_task'};

draco_lf_idx = zeros(1, 7);
draco_rf_idx = zeros(1, 7);
for i = 1: length(draco_lf_label)
    lf_idx = find(ismember(draco_joint_label, draco_lf_label(i)));
    rf_idx = find(ismember(draco_joint_label, draco_rf_label(i)));
    draco_lf_idx(i) = lf_idx;
    draco_rf_idx(i) = rf_idx;
end

%source functions
mfilepath = fileparts(which('plot_task'));
addpath(fullfile(erase(mfilepath, '/draco')));
helper_functions

%% wbc time post processing
num_wbc_data = length(rf_rf_cmd);
wbc_time = time(end-num_wbc_data+1:end);

%%
num_fig = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Icp task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
for i = 1:2
    subplot(2,1,i);
        j = j + 1;
        plot(wbc_time, des_icp(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, icp_est(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_icp(j,:), icp_est(j,:)]);
        max_val = max([des_icp(j,:), icp_est(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xy_label(j))
    sgtitle('ICP XY Task', 'FontSize', 30)
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
for i = 1:2
    subplot(2,1,i);
        j = j + 1;
        plot(wbc_time, local_des_icp(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_act_icp(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_icp(j,:), local_act_icp(j,:)]);
        max_val = max([local_des_icp(j,:), local_act_icp(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xy_label(j))
    sgtitle('ICP XY Task in LOCAL', 'FontSize', 30)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%com xy task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:4
    subplot(2,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, des_com_xy_pos(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_com_xy_pos(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_com_xy_pos(j,:), act_com_xy_pos(j,:)]);
        max_val = max([des_com_xy_pos(j,:), act_com_xy_pos(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xy_label(j))
    else
        k = k + 1;
        plot(wbc_time, des_com_xy_vel(k, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_com_xy_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_com_xy_vel(k,:), act_com_xy_vel(k,:)]);
        max_val = max([des_com_xy_vel(k,:), act_com_xy_vel(k,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xy_dot_label(k))
    end
    sgtitle('CoM XY Task', 'FontSize', 30)
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:4
    subplot(2,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, local_des_com_xy_pos(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_act_com_xy_pos(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_com_xy_pos(j,:), local_act_com_xy_pos(j,:)]);
        max_val = max([local_des_com_xy_pos(j,:), local_act_com_xy_pos(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xy_label(j))
    else
        k = k + 1;
        plot(wbc_time, local_des_com_xy_vel(k, :), 'r', 'LineWidth', 3);
         hold on
        plot(wbc_time, local_act_com_xy_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_com_xy_vel(k,:), local_act_com_xy_vel(k,:)]);
        max_val = max([local_des_com_xy_vel(k,:), local_act_com_xy_vel(k,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xy_dot_label(k))
    end
    sgtitle('CoM XY Task in LOCAL', 'FontSize', 30)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%com z task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1 : 2
    if i == 1
        subplot(1,2,i)
        plot(wbc_time, des_com_z_pos, 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_com_z_pos, 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_com_z_pos, act_com_z_pos]);
        max_val = max([des_com_z_pos, act_com_z_pos]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel("z")
    else
        subplot(1,2,i)
        plot(wbc_time, des_com_z_vel, 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_com_z_vel, 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_com_z_vel, act_com_z_vel]);
        max_val = max([des_com_z_vel, act_com_z_vel]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel('z_{dot}')
    end
    sgtitle('CoM Z Task', 'FontSize', 30)
end

figure(num_fig)
num_fig = num_fig + 1;
for i = 1 : 2
    if i == 1
        subplot(1,2,i)
        plot(wbc_time, local_des_com_z_pos, 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_act_com_z_pos, 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_com_z_pos, local_act_com_z_pos]);
        max_val = max([local_des_com_z_pos, local_act_com_z_pos]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel("z")
    else
        subplot(1,2,i)
        plot(wbc_time, local_des_com_z_vel, 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_act_com_z_vel, 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_com_z_vel, local_act_com_z_vel]);
        max_val = max([local_des_com_z_vel, local_act_com_z_vel]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel('z_{dot}')
    end
    sgtitle('CoM Z Task in LOCAL', 'FontSize', 30)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%CAM task in GLOBAL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
for i = 1:3
    subplot(3,1,i);
    plot(wbc_time, des_cam(i, :), 'r', 'LineWidth', 3);
    hold on
    plot(wbc_time, act_cam(i, :), 'b', 'LineWidth', 2);
    grid on
    min_val = min([des_cam(i,:), act_cam(i,:)]);
    max_val = max([des_cam(i,:), act_cam(i,:)]);
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(xyz_label(i))
    sgtitle('Centroidal Angular Momentum', 'FontSize', 30)
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
for i = 1:3
    subplot(3,1,i);
    plot(wbc_time, local_des_cam(i, :), 'r', 'LineWidth', 3);
    hold on
    plot(wbc_time, local_act_cam(i, :), 'b', 'LineWidth', 2);
    grid on
    min_val = min([local_des_cam(i,:), local_act_cam(i,:)]);
    max_val = max([local_des_cam(i,:), local_act_cam(i,:)]);
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(xyz_label(i))
    sgtitle('Centroidal Angular Momentum in LOCAL', 'FontSize', 30)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%torso ori task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[row, col] = size(des_torso_ori_pos);
torso_ori_des_quat = zeros(row, col);
torso_ori_des_quat(1, :) = des_torso_ori_pos(4, :);
torso_ori_des_quat(2:4, :) = des_torso_ori_pos(1:3, :);

torso_ori_act_quat = zeros(row, col);
torso_ori_act_quat(1, :) = act_torso_ori_pos(4, :);
torso_ori_act_quat(2:4, :) = act_torso_ori_pos(1:3, :);

torso_ori_des_euler_xyz = zeros(3, col);
torso_ori_act_euler_xyz = zeros(3, col);
for i = 1: col
    torso_ori_des_euler_xyz(:, i) = quat2eul(torso_ori_des_quat(:, i).', 'xyz');
    torso_ori_act_euler_xyz(:, i) = quat2eul(torso_ori_act_quat(:, i).', 'xyz');
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, torso_ori_des_euler_xyz(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, torso_ori_act_euler_xyz(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([torso_ori_des_euler_xyz(j,:), torso_ori_act_euler_xyz(j,:)]);
        max_val = max([torso_ori_des_euler_xyz(j,:), torso_ori_act_euler_xyz(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rpy_label(j))
    else
        k = k + 1;
        plot(wbc_time, des_torso_ori_vel(k, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_torso_ori_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_torso_ori_vel(j,:), act_torso_ori_vel(j,:)]);
        max_val = max([des_torso_ori_vel(j,:), act_torso_ori_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Torso ori task', 'FontSize', 30)
end

[row, col] = size(local_des_torso_ori_pos);
local_torso_ori_des_quat = zeros(row, col);
local_torso_ori_des_quat(1, :) = local_des_torso_ori_pos(4, :);
local_torso_ori_des_quat(2:4, :) = local_des_torso_ori_pos(1:3, :);

local_torso_ori_act_quat = zeros(row, col);
local_torso_ori_act_quat(1, :) = local_act_torso_ori_pos(4, :);
local_torso_ori_act_quat(2:4, :) = local_act_torso_ori_pos(1:3, :);

local_torso_ori_des_euler_xyz = zeros(3, col);
local_torso_ori_act_euler_xyz = zeros(3, col);
for i = 1: col
    local_torso_ori_des_euler_xyz(:, i) = quat2eul(local_torso_ori_des_quat(:, i).', 'xyz');
    local_torso_ori_act_euler_xyz(:, i) = quat2eul(local_torso_ori_act_quat(:, i).', 'xyz');
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, local_torso_ori_des_euler_xyz(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_torso_ori_act_euler_xyz(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_torso_ori_des_euler_xyz(j,:), local_torso_ori_act_euler_xyz(j,:)]);
        max_val = max([local_torso_ori_des_euler_xyz(j,:), local_torso_ori_act_euler_xyz(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rpy_label(j))
    else
        k = k + 1;
        plot(wbc_time, local_des_torso_ori_vel(k, :), 'r', 'LineWidth', 3);
         hold on
        plot(wbc_time, local_act_torso_ori_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_torso_ori_vel(j,:), local_act_torso_ori_vel(j,:)]);
        max_val = max([local_des_torso_ori_vel(j,:), local_act_torso_ori_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Torso ori task in LOCAL', 'FontSize', 30)
end

if b_include_lhand_task
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %left hand pos task
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(num_fig)
    num_fig = num_fig + 1;
    j = 0;
    k = 0;
    for i = 1:6
        subplot(3,2,i);
        if mod(i, 2) == 1
            j = j + 1;
            plot(wbc_time, des_lh_pos(j, :), 'r', 'LineWidth', 3);
            hold on
            plot(wbc_time, act_lh_pos(j, :), 'b', 'LineWidth', 2);
            grid on
            min_val = min([des_lh_pos(j,:), act_lh_pos(j,:)]);
            max_val = max([des_lh_pos(j,:), act_lh_pos(j,:)]);
            min_val = min_val - 0.1 * (max_val - min_val);
            max_val = max_val + 0.1 *(max_val - min_val);
            set_fig_opt()
            plot_phase(time, state, min_val, max_val, phase_color)
            xlabel('time')
            ylabel(xyz_label(j))
        else
            k = k + 1;
            plot(wbc_time, des_lh_vel(k, :), 'r', 'LineWidth', 3);
             hold on
            plot(wbc_time, act_lh_vel(k, :), 'b', 'LineWidth', 2);
            grid on
            min_val = min([des_lh_vel(j,:), act_lh_vel(j,:)]);
            max_val = max([des_lh_vel(j,:), act_lh_vel(j,:)]);
            min_val = min_val - 0.1 * (max_val - min_val);
            max_val = max_val + 0.1 *(max_val - min_val);
            set_fig_opt()
            plot_phase(time, state, min_val, max_val, phase_color)
            xlabel('time')
            ylabel(xyz_dot_label(k))
        end
        sgtitle('Left Hand Task', 'FontSize', 30)
    end
    
    % orientation
    [row, col] = size(des_lh_ori);
    des_lh_quat = zeros(row, col);
    des_lh_quat(1, :) = des_lh_ori(4, :);
    des_lh_quat(2:4, :) = des_lh_ori(1:3, :);
    
    act_lh_quat = zeros(row, col);
    act_lh_quat(1, :) = act_lf_ori(4, :);
    act_lh_quat(2:4, :) = act_lf_ori(1:3, :);
    
    des_lh_ori_euler_xyz = zeros(3, col);
    act_lh_ori_euler_xyz = zeros(3, col);
    for i = 1: col
        des_lh_ori_euler_xyz(:, i) = quat2eul(des_lh_quat(:, i).', 'xyz');
        act_lh_ori_euler_xyz(:, i) = quat2eul(act_lh_quat(:, i).', 'xyz');
    end
    
    figure(num_fig)
    num_fig = num_fig + 1;
    j = 0;
    k = 0;
    for i = 1:6
        subplot(3,2,i);
        if mod(i, 2) == 1
            j = j + 1;
            plot(wbc_time, des_lh_ori_euler_xyz(j, :), 'r', 'LineWidth', 3);
            hold on
            plot(wbc_time, act_lh_ori_euler_xyz(j, :), 'b', 'LineWidth', 2);
            grid on
            min_val = min([des_lh_ori_euler_xyz(j,:), act_lh_ori_euler_xyz(j,:)]);
            max_val = max([des_lh_ori_euler_xyz(j,:), act_lh_ori_euler_xyz(j,:)]);
            min_val = min_val - 0.1 * (max_val - min_val);
            max_val = max_val + 0.1 *(max_val - min_val);
            set_fig_opt()
            plot_phase(time, state, min_val, max_val, phase_color)
            xlabel('time')
            ylabel(rpy_label(j))
        else
            k = k + 1;
            plot(wbc_time, des_lh_ori_vel(k, :), 'r', 'LineWidth', 3);
             hold on
            plot(wbc_time, act_lh_ori_vel(k, :), 'b', 'LineWidth', 2);
            grid on
            min_val = min([des_lh_ori_vel(j,:), act_lh_ori_vel(j,:)]);
            max_val = max([des_lh_ori_vel(j,:), act_lh_ori_vel(j,:)]);
            min_val = min_val - 0.1 * (max_val - min_val);
            max_val = max_val + 0.1 *(max_val - min_val);
            set_fig_opt()
            plot_phase(time, state, min_val, max_val, phase_color)
            xlabel('time')
            ylabel(xyz_dot_label(k))
        end
        sgtitle('Left Hand ori task', 'FontSize', 30)
    end
    
    % Local
    figure(num_fig)
    num_fig = num_fig + 1;
    j = 0;
    k = 0;
    for i = 1:6
        subplot(3,2,i);
        if mod(i, 2) == 1
            j = j + 1;
            plot(wbc_time, local_des_lh_pos(j, :), 'r', 'LineWidth', 3);
            hold on
            plot(wbc_time, local_act_lh_pos(j, :), 'b', 'LineWidth', 2);
            grid on
            min_val = min([local_des_lh_pos(j,:), local_act_lh_pos(j,:)]);
            max_val = max([local_des_lh_pos(j,:), local_act_lh_pos(j,:)]);
            min_val = min_val - 0.1 * (max_val - min_val);
            max_val = max_val + 0.1 *(max_val - min_val);
            set_fig_opt()
            plot_phase(time, state, min_val, max_val, phase_color)
            xlabel('time')
            ylabel(xyz_label(j))
        else
            k = k + 1;
            plot(wbc_time, local_des_lh_vel(k, :), 'r', 'LineWidth', 3);
             hold on
            plot(wbc_time, local_act_lh_vel(k, :), 'b', 'LineWidth', 2);
            grid on
            min_val = min([local_des_lh_vel(j,:), local_act_lh_vel(j,:)]);
            max_val = max([local_des_lh_vel(j,:), local_act_lh_vel(j,:)]);
            min_val = min_val - 0.1 * (max_val - min_val);
            max_val = max_val + 0.1 *(max_val - min_val);
            set_fig_opt()
            plot_phase(time, state, min_val, max_val, phase_color)
            xlabel('time')
            ylabel(xyz_dot_label(k))
        end
        sgtitle('Left Hand Task in LOCAL', 'FontSize', 30)
    end
    
    % orientation
    [row, col] = size(local_des_lh_ori);
    local_des_lh_quat = zeros(row, col);
    local_des_lh_quat(1, :) = local_des_lh_ori(4, :);
    local_des_lh_quat(2:4, :) = local_des_lh_ori(1:3, :);
    
    local_act_lh_quat = zeros(row, col);
    local_act_lh_quat(1, :) = local_act_lf_ori(4, :);
    local_act_lh_quat(2:4, :) = local_act_lf_ori(1:3, :);
    
    local_des_lh_ori_euler_xyz = zeros(3, col);
    local_act_lh_ori_euler_xyz = zeros(3, col);
    for i = 1: col
        local_des_lh_ori_euler_xyz(:, i) = quat2eul(local_des_lh_quat(:, i).', 'xyz');
        local_act_lh_ori_euler_xyz(:, i) = quat2eul(local_act_lh_quat(:, i).', 'xyz');
    end
    
    figure(num_fig)
    num_fig = num_fig + 1;
    j = 0;
    k = 0;
    for i = 1:6
        subplot(3,2,i);
        if mod(i, 2) == 1
            j = j + 1;
            plot(wbc_time, local_des_lh_ori_euler_xyz(j, :), 'r', 'LineWidth', 3);
            hold on
            plot(wbc_time, local_act_lh_ori_euler_xyz(j, :), 'b', 'LineWidth', 2);
            grid on
            min_val = min([local_des_lh_ori_euler_xyz(j,:), local_act_lh_ori_euler_xyz(j,:)]);
            max_val = max([local_des_lh_ori_euler_xyz(j,:), local_act_lh_ori_euler_xyz(j,:)]);
            min_val = min_val - 0.1 * (max_val - min_val);
            max_val = max_val + 0.1 *(max_val - min_val);
            set_fig_opt()
            plot_phase(time, state, min_val, max_val, phase_color)
            xlabel('time')
            ylabel(rpy_label(j))
        else
            k = k + 1;
            plot(wbc_time, local_des_lh_ori_vel(k, :), 'r', 'LineWidth', 3);
             hold on
            plot(wbc_time, local_act_lh_ori_vel(k, :), 'b', 'LineWidth', 2);
            grid on
            min_val = min([local_des_lh_ori_vel(j,:), local_act_lh_ori_vel(j,:)]);
            max_val = max([local_des_lh_ori_vel(j,:), local_act_lh_ori_vel(j,:)]);
            min_val = min_val - 0.1 * (max_val - min_val);
            max_val = max_val + 0.1 *(max_val - min_val);
            set_fig_opt()
            plot_phase(time, state, min_val, max_val, phase_color)
            xlabel('time')
            ylabel(xyz_dot_label(k))
        end
        sgtitle('Left Hand ori task in LOCAL', 'FontSize', 30)
    end
end

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
        plot(wbc_time, des_lf_pos(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_lf_pos(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_lf_pos(j,:), act_lf_pos(j,:)]);
        max_val = max([des_lf_pos(j,:), act_lf_pos(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_label(j))
    else
        k = k + 1;
        plot(wbc_time, des_lf_vel(k, :), 'r', 'LineWidth', 3);
         hold on
        plot(wbc_time, act_lf_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_lf_vel(j,:), act_lf_vel(j,:)]);
        max_val = max([des_lf_vel(j,:), act_lf_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Left Foot Task', 'FontSize', 30)
end

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
    sgtitle('Left Foot Task in LOCAL', 'FontSize', 30)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%left foot ori task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[row, col] = size(des_lf_ori);
des_lf_quat = zeros(row, col);
des_lf_quat(1, :) = des_lf_ori(4, :);
des_lf_quat(2:4, :) = des_lf_ori(1:3, :);

act_lf_quat = zeros(row, col);
act_lf_quat(1, :) = act_lf_ori(4, :);
act_lf_quat(2:4, :) = act_lf_ori(1:3, :);

des_lf_ori_euler_xyz = zeros(3, col);
act_lf_ori_euler_xyz = zeros(3, col);
for i = 1: col
    des_lf_ori_euler_xyz(:, i) = quat2eul(des_lf_quat(:, i).', 'xyz');
    act_lf_ori_euler_xyz(:, i) = quat2eul(act_lf_quat(:, i).', 'xyz');
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, des_lf_ori_euler_xyz(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_lf_ori_euler_xyz(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_lf_ori_euler_xyz(j,:), act_lf_ori_euler_xyz(j,:)]);
        max_val = max([des_lf_ori_euler_xyz(j,:), act_lf_ori_euler_xyz(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rpy_label(j))
    else
        k = k + 1;
        plot(wbc_time, des_lf_ori_vel(k, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_lf_ori_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_lf_ori_vel(j,:), act_lf_ori_vel(j,:)]);
        max_val = max([des_lf_ori_vel(j,:), act_lf_ori_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Left foot ori task', 'FontSize', 30)
end

[row, col] = size(local_des_lf_ori);
local_des_lf_quat = zeros(row, col);
local_des_lf_quat(1, :) = local_des_lf_ori(4, :);
local_des_lf_quat(2:4, :) = local_des_lf_ori(1:3, :);

local_act_lf_quat = zeros(row, col);
local_act_lf_quat(1, :) = local_act_lf_ori(4, :);
local_act_lf_quat(2:4, :) = local_act_lf_ori(1:3, :);

local_des_lf_ori_euler_xyz = zeros(3, col);
local_act_lf_ori_euler_xyz = zeros(3, col);
for i = 1: col
    local_des_lf_ori_euler_xyz(:, i) = quat2eul(local_des_lf_quat(:, i).', 'xyz');
    local_act_lf_ori_euler_xyz(:, i) = quat2eul(local_act_lf_quat(:, i).', 'xyz');
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, local_des_lf_ori_euler_xyz(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_act_lf_ori_euler_xyz(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_lf_ori_euler_xyz(j,:), local_act_lf_ori_euler_xyz(j,:)]);
        max_val = max([local_des_lf_ori_euler_xyz(j,:), local_act_lf_ori_euler_xyz(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rpy_label(j))
    else
        k = k + 1;
        plot(wbc_time, local_des_lf_ori_vel(k, :), 'r', 'LineWidth', 3);
         hold on
        plot(wbc_time, local_act_lf_ori_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_lf_ori_vel(j,:), local_act_lf_ori_vel(j,:)]);
        max_val = max([local_des_lf_ori_vel(j,:), local_act_lf_ori_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Left Foot ori task in LOCAL', 'FontSize', 30)
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
    sgtitle('Right Foot Task', 'FontSize', 30)
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, local_des_rf_pos(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_act_rf_pos(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_rf_pos(j,:), local_act_rf_pos(j,:)]);
        max_val = max([local_des_rf_pos(j,:), local_act_rf_pos(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_label(j))
    else
        k = k + 1;
        plot(wbc_time, local_des_rf_vel(k, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_act_rf_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_rf_vel(j,:), local_act_rf_vel(j,:)]);
        max_val = max([local_des_rf_vel(j,:), local_act_rf_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Right Foot Task in LOCAL', 'FontSize', 30)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%right foot ori task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[row, col] = size(des_rf_ori);
des_rf_quat = zeros(row, col);
des_rf_quat(1, :) = des_rf_ori(4, :);
des_rf_quat(2:4, :) = des_rf_ori(1:3, :);

act_rf_quat = zeros(row, col);
act_rf_quat(1, :) = act_rf_ori(4, :);
act_rf_quat(2:4, :) = act_rf_ori(1:3, :);

des_rf_ori_euler_xyz = zeros(3, col);
act_rf_ori_euler_xyz = zeros(3, col);
for i = 1: col
    des_rf_ori_euler_xyz(:, i) = quat2eul(des_rf_quat(:, i).', 'xyz');
    act_rf_ori_euler_xyz(:, i) = quat2eul(act_rf_quat(:, i).', 'xyz');
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, des_rf_ori_euler_xyz(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, act_rf_ori_euler_xyz(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_rf_ori_euler_xyz(j,:), act_rf_ori_euler_xyz(j,:)]);
        max_val = max([des_rf_ori_euler_xyz(j,:), act_rf_ori_euler_xyz(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rpy_label(j))
    else
        k = k + 1;
        plot(wbc_time, des_rf_ori_vel(k, :), 'r', 'LineWidth', 3);
         hold on
        plot(wbc_time, act_rf_ori_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_rf_ori_vel(j,:), act_rf_ori_vel(j,:)]);
        max_val = max([des_rf_ori_vel(j,:), act_rf_ori_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Right foot ori task', 'FontSize', 30)
end

[row, col] = size(local_des_rf_ori);
local_des_rf_quat = zeros(row, col);
local_des_rf_quat(1, :) = local_des_rf_ori(4, :);
local_des_rf_quat(2:4, :) = local_des_rf_ori(1:3, :);

local_act_rf_quat = zeros(row, col);
local_act_rf_quat(1, :) = local_act_rf_ori(4, :);
local_act_rf_quat(2:4, :) = local_act_rf_ori(1:3, :);

local_des_rf_ori_euler_xyz = zeros(3, col);
local_act_rf_ori_euler_xyz = zeros(3, col);
for i = 1: col
    local_des_rf_ori_euler_xyz(:, i) = quat2eul(local_des_rf_quat(:, i).', 'xyz');
    local_act_rf_ori_euler_xyz(:, i) = quat2eul(local_act_rf_quat(:, i).', 'xyz');
end

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(wbc_time, local_des_rf_ori_euler_xyz(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(wbc_time, local_act_rf_ori_euler_xyz(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_rf_ori_euler_xyz(j,:), local_act_rf_ori_euler_xyz(j,:)]);
        max_val = max([local_des_rf_ori_euler_xyz(j,:), local_act_rf_ori_euler_xyz(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rpy_label(j))
    else
        k = k + 1;
        plot(wbc_time, local_des_rf_ori_vel(k, :), 'r', 'LineWidth', 3);
         hold on
        plot(wbc_time, local_act_rf_ori_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([local_des_rf_ori_vel(j,:), local_act_rf_ori_vel(j,:)]);
        max_val = max([local_des_rf_ori_vel(j,:), local_act_rf_ori_vel(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Right Foot ori task in LOCAL', 'FontSize', 30)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Whole-body control commands
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% reaction force cmd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:12
    ax(i) = subplot(6,2,i);
    if mod(i,2) == 1
        j = j + 1;
        plot(wbc_time, lf_rf_cmd(j, :), 'r', 'LineWidth', 2);
        grid on
        hold on
        plot(wbc_time, des_rf_lfoot(j, :), 'k', 'LineWidth', 2);
        min_val = min([lf_rf_cmd(j, :)]);
        max_val = max([lf_rf_cmd(j, :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
%       set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rf_label(j))
        if j == 1
            title('left foot reaction force in LOCAL', 'FontSize',30)
        end
    else
        k = k + 1;
        plot(wbc_time, rf_rf_cmd(k, :), 'r', 'LineWidth', 2);
        grid on
        hold on
        plot(wbc_time, des_rf_rfoot(k, :), 'k', 'LineWidth', 2);
        min_val = min([rf_rf_cmd(k, :)]);
        max_val = max([rf_rf_cmd(k, :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
%       set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rf_label(k))
         if j == 1
            title('right foot reaction force in LOCAL', 'FontSize', 30)
        end
    end
    legend('cmd','des')
end
linkaxes(ax,'x')

figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:12
    ax(i)=subplot(6,2,i);
    if mod(i,2) == 1
        j = j + 1;
        plot(wbc_time, lf_rf_cmd_global(j, :), 'r', 'LineWidth', 2);
        grid on
        hold on
        min_val = min([lf_rf_cmd_global(j, :)]);
        max_val = max([lf_rf_cmd_global(j, :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
%       set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rf_label(j))
        if j == 1
            title('left foot reaction force in GLOBAL', 'FontSize',30)
        end
    else
        k = k + 1;
        plot(wbc_time, rf_rf_cmd_global(k, :), 'r', 'LineWidth', 2);
        grid on
        hold on
        min_val = min([rf_rf_cmd_global(k, :)]);
        max_val = max([rf_rf_cmd_global(k, :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
%       set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(rf_label(k))
         if j == 1
            title('right foot reaction force in GLOBAL', 'FontSize', 30)
        end
    end
    legend('cmd')
end
linkaxes(ax,'x')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% floating base qddot cmd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:6
    subplot(6,1,i);
    plot(wbc_time, fb_qddot_cmd(i, :), 'b', 'LineWidth', 2);
    grid on
    hold on
    plot(wbc_time, corrected_fb_qddot_cmd(i, :), 'r', 'LineWidth', 2);
    min_val = min([fb_qddot_cmd(i, :), corrected_fb_qddot_cmd(i, :)]);
    max_val = max([fb_qddot_cmd(i, :), corrected_fb_qddot_cmd(i, :)]);
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
%   set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(fb_qddot_label(i))
    sgtitle('floating base qddot cmd', 'FontSize', 30)
    legend('before correction', 'after correction')
end

figure(num_fig)
num_fig = num_fig + 1;
for i = 1:6
    subplot(6,1,i);
    plot(wbc_time, fb_qddot_cmd(i, :) - corrected_fb_qddot_cmd(i, :), 'b', 'LineWidth', 2);
    grid on
    min_val = min(fb_qddot_cmd(i, :) - corrected_fb_qddot_cmd(i, :));
    max_val = max(fb_qddot_cmd(i, :) - corrected_fb_qddot_cmd(i, :));
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
%   set_fig_opt()
%     plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(fb_qddot_label(i))
    sgtitle('floating base qddot error', 'FontSize', 30)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% foot qddot cmd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:14
    subplot(7,2,i);
    if mod(i,2) == 1
        j = j + 1;
        plot(wbc_time, joint_acc_cmd(draco_lf_idx(j), :), 'r', 'LineWidth', 2);
        grid on
        hold on
        min_val = min([joint_acc_cmd(draco_lf_idx(j), :)]);
        max_val = max([joint_acc_cmd(draco_lf_idx(j), :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
    %   set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(draco_lf_label(j))
        if j == 1
            title('left foot qddot cmd', 'FontSize',30)
        end
    else
        k = k + 1;
        plot(wbc_time, joint_acc_cmd(draco_rf_idx(k), :), 'r', 'LineWidth', 2);
        grid on
        hold on
        min_val = min([joint_acc_cmd(draco_rf_idx(k), :)]);
        max_val = max([joint_acc_cmd(draco_rf_idx(k), :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
    %   set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(draco_rf_label(k))
         if j == 1
            title('right foot qddot cmd', 'FontSize', 30)
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% foot torque cmd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:14
    subplot(7,2,i);
    if mod(i,2) == 1
        j = j + 1;
        plot(wbc_time, joint_trq_cmd(draco_lf_idx(j), :), 'r', 'LineWidth', 2);
        grid on
        hold on
        min_val = min([joint_trq_cmd(draco_lf_idx(j), :)]);
        max_val = max([joint_trq_cmd(draco_lf_idx(j), :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
    %   set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(draco_lf_label(j))
        if j == 1
            title('left foot trq cmd', 'FontSize',30)
        end
    else
        k = k + 1;
        plot(wbc_time, joint_trq_cmd(draco_rf_idx(k), :), 'r', 'LineWidth', 2);
        grid on
        hold on
        min_val = min([joint_trq_cmd(draco_rf_idx(k), :)]);
        max_val = max([joint_trq_cmd(draco_rf_idx(k), :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
    %   set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(draco_rf_label(k))
         if j == 1
            title('right foot trq cmd', 'FontSize', 30)
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lower body joint position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:14
    ax(i) = subplot(7,2,i);
    if mod(i,2) == 1
        j = j + 1;
        plot(time, joint_pos_cmd(draco_lf_idx(j), :), 'r', 'LineWidth', 3);
        grid on
        hold on
        plot(time, joint_pos_act(draco_lf_idx(j), :), 'b', 'LineWidth', 2);
        min_val = min([joint_pos_cmd(draco_lf_idx(j), :), joint_pos_act(draco_lf_idx(j), :)]);
        max_val = max([joint_pos_cmd(draco_lf_idx(j), :), joint_pos_act(draco_lf_idx(j), :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
    %   set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(draco_lf_label(j))
        if j == 1
            title('left foot jpos data', 'FontSize',30)
        end
    else
        k = k + 1;
        plot(time, joint_pos_cmd(draco_rf_idx(k), :), 'r', 'LineWidth', 3);
        grid on
        hold on
        plot(time, joint_pos_act(draco_rf_idx(k), :), 'b', 'LineWidth', 2);
        min_val = min([joint_pos_cmd(draco_rf_idx(k), :), joint_pos_act(draco_rf_idx(k), :)]);
        max_val = max([joint_pos_cmd(draco_rf_idx(k), :), joint_pos_act(draco_rf_idx(k), :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
    %   set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(draco_rf_label(k))
         if j == 1
            title('right foot jpos data', 'FontSize', 30)
        end
    end
end
linkaxes(ax, 'x')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lower body joint velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:14
    ax(i) = subplot(7,2,i);
    if mod(i,2) == 1
        j = j + 1;
        plot(time, joint_vel_cmd(draco_lf_idx(j), :), 'r', 'LineWidth', 3);
        grid on
        hold on
        plot(time, joint_vel_act(draco_lf_idx(j), :), 'b', 'LineWidth', 2);
        min_val = min([joint_vel_cmd(draco_lf_idx(j), :), joint_vel_act(draco_lf_idx(j), :)]);
        max_val = max([joint_vel_cmd(draco_lf_idx(j), :), joint_vel_act(draco_lf_idx(j), :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
    %   set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(draco_lf_label(j))
        if j == 1
            title('left foot jvel data', 'FontSize',30)
        end
    else
        k = k + 1;
        plot(time, joint_vel_cmd(draco_rf_idx(k), :), 'r', 'LineWidth', 3);
        grid on
        hold on
        plot(time, joint_vel_act(draco_rf_idx(k), :), 'b', 'LineWidth', 2);
        min_val = min([joint_vel_cmd(draco_rf_idx(j), :), joint_vel_act(draco_rf_idx(j), :)]);
        max_val = max([joint_vel_cmd(draco_rf_idx(j), :), joint_vel_act(draco_rf_idx(j), :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
    %   set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(draco_rf_label(k))
         if j == 1
            title('right foot jvel data', 'FontSize', 30)
        end
    end
end
linkaxes(ax,'x')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% contact constraint
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:12
    ax(i) = subplot(6,2,i);
    if mod(i,2) == 1
        j = j + 1;
        plot(wbc_time, xc_ddot(j, :), 'r', 'LineWidth', 2);
        grid on
        hold on
        min_val = min([xc_ddot(j, :)]);
        max_val = max([xc_ddot(j, :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
%       set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(contact_acc_label(j))
        if j == 1
            title('left foot contact constraint in LOCAL', 'FontSize',30)
        end
    else
        k = k + 1;
        plot(wbc_time, xc_ddot(6+k, :), 'r', 'LineWidth', 2);
        grid on
        hold on
        min_val = min([xc_ddot(6+k, :)]);
        max_val = max([xc_ddot(6+k, :)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
%       set_fig_opt()
        plot_phase(time, state, min_val, max_val, phase_color)
        xlabel('time')
        ylabel(contact_acc_label(k))
         if j == 1
            title('right foot contact constraint in LOCAL', 'FontSize', 30)
        end
    end
end
linkaxes(ax,'x')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% QP cost
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    ax(i) = subplot(1,3,i);
    if i == 1
        plot(wbc_time, delta_qddot_cost, 'b', 'LineWidth', 3);
        grid on
        hold on
        min_val = min(delta_qddot_cost);
        max_val = max(delta_qddot_cost);
        plot_phase(time, state, min_val, max_val, phase_color);
        xlabel('time')
        title('delta_qddot_cost', 'FontSize', 30)
    elseif i == 2
        plot(wbc_time, delta_rf_cost, 'b', 'LineWidth', 3);
        grid on
        hold on
        min_val = min(delta_rf_cost);
        max_val = max(delta_rf_cost);
        plot_phase(time, state, min_val, max_val, phase_color);
        xlabel('time')
        title('delta_rf_cost', 'FontSize', 30)
    elseif i ==3
        plot(wbc_time, xc_ddot_cost, 'b', 'LineWidth', 3);
        grid on
        hold on
        min_val = min(xc_ddot_cost);
        max_val = max(xc_ddot_cost);
        plot_phase(time, state, min_val, max_val, phase_color);
        xlabel('time')
        title('xc_ddot_cost', 'FontSize', 30) 
    end
end
linkaxes(ax,'x')
