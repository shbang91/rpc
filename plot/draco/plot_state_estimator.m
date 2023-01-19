close all;
clear;
clc;

addpath("/tmp")

d = dir("/tmp/draco_icp_data*.mat");
dd = dir("/tmp/draco_controller_data*.mat");
ddd = dir("/tmp/draco_state_estimator*.mat");

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

quat_label = ["q_x", "q_y", "q_z", "q_w"];
xyz_label = ["x","y","z"];
xyz_dot_label = ["x_{dot}", "y_{dot}", "z_{dot}"];
rpy_label = ["roll", "pitch", "yaw"];
ang_vel_label = ["wx_{dot}", "wy_{dot}", "wz_{dot}"];
xy_label = ["x","y"];
xy_dot_label = ["x_{dot}", "y_{dot}"];
rf_label = ["trq_{x}", "trq_{y}", "trq_{z}", "f_{x}", "f_{y}", "f_{z}"];
fb_qddot_label = ["x_{ddot}", "y_{ddot}", "z_{ddot}", "wx_{dot}", "wy_{dot}", "wz_{dot}"];

phase_color = ["#A2142F", "#FF0000","#00FF00", "#0000FF","#00FFFF", "#FF00FF", "#FFFF00",	"#0072BD", "#D95319", "#EDB120"	, "#7E2F8E", "#77AC30", "#A2142F"];

%source functions
mfilepath = fileparts(which('plot_state_estimator'));
addpath(fullfile(erase(mfilepath, '/draco')));
helper_functions

%%
num_fig = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%floating base estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    subplot(3,1, i);
    plot(time, base_joint_pos_est(i, :), 'b', 'LineWidth',2);
    grid on
    hold on
    min_val = min(base_joint_pos_est(i,:));
    max_val = max(base_joint_pos_est(i,:));
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(xyz_label(i))
    sgtitle('base joint pos est', 'FontSize', 30)
end

figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    subplot(3, 1, i);
    plot(time, base_joint_rpy_est(i, :), 'b', 'LineWidth',2);
    grid on
    hold on
    min_val = min(base_joint_rpy_est(i,:));
    max_val = max(base_joint_rpy_est(i,:));
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(rpy_label(i))
    sgtitle('base joint rpy est', 'FontSize', 30)
end

figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    subplot(3, 1, i);
    plot(time, base_joint_lin_vel_est(i, :), 'b', 'LineWidth',2);
    grid on
    hold on
    min_val = min(base_joint_lin_vel_est(i,:));
    max_val = max(base_joint_lin_vel_est(i,:));
    min_val = min_val - 0.1 * (max_val - min_val);
    max_val = max_val + 0.1 *(max_val - min_val);
    set_fig_opt()
    plot_phase(time, state, min_val, max_val, phase_color)
    xlabel('time')
    ylabel(xyz_label(i))
    sgtitle('base joint lin vel est', 'FontSize', 30)
end 

figure(num_fig)
num_fig = num_fig + 1;
for i = 1:3
    subplot(3, 1, i);
    plot(time, base_joint_ang_vel_est(i, :), 'b', 'LineWidth',2);
    grid on
    hold on
    min_val = min(base_joint_ang_vel_est(i,:));
    max_val = max(base_joint_ang_vel_est(i,:));
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
     subplot(3,1,i);
        plot(time, com_vel_raw(i, :), 'k', 'LineWidth', 3);
        hold on
        plot(time, com_vel_est(i, :), 'r', 'LineWidth', 2);
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% icp est
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
j = 0;
k = 0;
for i = 1:4
    subplot(2, 2, i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time, icp_est(j, :), 'b', 'LineWidth',2);
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
        plot(time, icp_vel_est(k, :), 'b', 'LineWidth', 2);
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%icp err plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(num_fig)
num_fig = num_fig + 1;
for i = 1:2
    subplot(2,1,i);
        plot(time, icp_error_raw(i, :), 'k', 'LineWidth', 3);
        hold on
        plot(time, icp_avg_err(i, :), 'r', 'LineWidth', 2);
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