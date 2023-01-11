close all;
clear;
clc;

addpath("/tmp")

d = dir("/tmp/icp_err*.mat");
dd = dir("/tmp/task_ref*.mat");

[tmp, i] = max([d.datenum]);
fprintf('loading %s \n', d(i).name)
load(d(i).name)

[tmp, i] = max([dd.datenum]);
load(dd(i).name, 'time')

quat_label = ["q_x", "q_y", "q_z", "q_w"];
xyz_label = ["x","y","z"];
xyz_dot_label = ["x_{dot}", "y_{dot}", "z_{dot}"];
xy_label = ["x","y"];
xy_dot_label = ["x_{dot}", "y_{dot}"];
rf_label = ["trq_{x}", "trq_{y}", "trq_{z}", "f_{x}", "f_{y}", "f_{z}"];
fb_qddot_label = ["x_{ddot}", "y_{ddot}", "z_{ddot}", "wx_{dot}", "wy_{dot}", "wz_{dot}"];

%%
%icp err plot
figure(1)
for i = 1:2
    subplot(2,1,i);
        plot(time, icp_error_raw(i, :), 'k', 'LineWidth', 3);
        hold on
        plot(time, icp_avg_err(i, :), 'r', 'LineWidth', 2);
        grid on
        min_val = min([icp_error_raw(i,:), icp_avg_err(i,:)]);
        max_val = max([icp_error_raw(i,:), icp_avg_err(i,:)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 *(max_val - min_val))
        xlabel('time')
        ylabel(xy_label(i))
    sgtitle('icp_integrator', 'FontSize', 30)
end



function [] = set_fig_opt(min,max)
    set(gca, 'LineWidth', 3)
    set(gca, 'TickLabelInterpreter', 'latex')
    set(gca, 'FontSize', 30)
    set(gca, 'Color', 'white')
    ylim([min, max])
end
