close all;
clear;
clc;

addpath("/tmp")

d = dir("/tmp/task_ref*.mat");

[tmp, i] = max([d.datenum]);
fprintf('loading %s \n', d(i).name)
load(d(i).name)

quat_label = ["q_x", "q_y", "q_z", "q_w"];
xyz_label = ["x","y","z"];
xyz_dot_label = ["x_{dot}", "y_{dot}", "z_{dot}"];
xy_label = ["x","y"];
xy_dot_label = ["x_{dot}", "y_{dot}"];

%%
%com xy task
figure(1)
j = 0;
k = 0;
for i = 1:4
    subplot(2,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time, des_com_xy_pos(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(time, act_com_xy_pos(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_com_xy_pos(j,:), act_com_xy_pos(j,:)]);
        max_val = max([des_com_xy_pos(j,:), act_com_xy_pos(j,:)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 *(max_val - min_val))
        xlabel('time')
        ylabel(xy_label(j))
    else
        k = k + 1;
        plot(time, des_com_xy_vel(k, :), 'r', 'LineWidth', 3);
         hold on
        plot(time, act_com_xy_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_com_xy_vel(k,:), act_com_xy_vel(k,:)]);
        max_val = max([des_com_xy_vel(k,:), act_com_xy_vel(k,:)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 *(max_val - min_val))
        xlabel('time')
        ylabel(xy_dot_label(k))
    end
    sgtitle('CoM XY Task', 'FontSize', 30)
end

%com z task
figure(2)
subplot(1,2,1)
plot(time, des_com_z_pos, 'r', 'LineWidth', 3);
hold on
plot(time, act_com_z_pos, 'b', 'LineWidth', 2);
grid on
min_val = min([des_com_z_pos, act_com_z_pos]);
max_val = max([des_com_z_pos, act_com_z_pos]);
set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 *(max_val - min_val))
xlabel('time')
ylabel("z")
subplot(1,2,2)
plot(time, des_com_z_vel, 'r', 'LineWidth', 3);
hold on
plot(time, act_com_z_vel, 'b', 'LineWidth', 2);
grid on
min_val = min([des_com_z_vel, act_com_z_vel]);
max_val = max([des_com_z_vel, act_com_z_vel]);
set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 *(max_val - min_val))
xlabel('time')
ylabel('z_{dot}')
sgtitle('CoM Z Task', 'FontSize', 30)

%left foot task
figure(3)
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time, des_lf_pos(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(time, act_lf_pos(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_lf_pos(j,:), act_lf_pos(j,:)]);
        max_val = max([des_lf_pos(j,:), act_lf_pos(j,:)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 *(max_val - min_val))
        xlabel('time')
        ylabel(xyz_label(j))
    else
        k = k + 1;
        plot(time, des_lf_vel(k, :), 'r', 'LineWidth', 3);
         hold on
        plot(time, act_lf_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_lf_vel(j,:), act_lf_vel(j,:)]);
        max_val = max([des_lf_vel(j,:), act_lf_vel(j,:)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 *(max_val - min_val))
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Left Foot Task', 'FontSize', 30)
end

%right foot task
figure(4)
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time, des_rf_pos(j, :), 'r', 'LineWidth', 3);
        hold on
        plot(time, act_rf_pos(j, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_rf_pos(j,:), act_rf_pos(j,:)]);
        max_val = max([des_rf_pos(j,:), act_rf_pos(j,:)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 *(max_val - min_val))
        xlabel('time')
        ylabel(xyz_label(j))
    else
        k = k + 1;
        plot(time, des_rf_vel(k, :), 'r', 'LineWidth', 3);
        hold on
        plot(time, act_rf_vel(k, :), 'b', 'LineWidth', 2);
        grid on
        min_val = min([des_rf_vel(j,:), act_rf_vel(j,:)]);
        max_val = max([des_rf_vel(j,:), act_rf_vel(j,:)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 *(max_val - min_val))
        xlabel('time')
        ylabel(xyz_dot_label(k))
    end
    sgtitle('Right Foot Task', 'FontSize', 30)
end


function [] = set_fig_opt(min,max)
    set(gca, 'LineWidth', 3)
    set(gca, 'TickLabelInterpreter', 'latex')
    set(gca, 'FontSize', 30)
    set(gca, 'Color', 'white')
    ylim([min, max])
end
