close all;
clear;
clc;

addpath("/tmp")

d = dir("/tmp/task_ref*.mat");

[tmp, i] = max([d.datenum]);
fprintf('loading %s \n', d(i).name)
load(d(i).name)

%%
quat_label = ["q_x", "q_y", "q_z", "q_w"];
xyz_label = ["x","y","z"];
xyz_dot_label = ["x_{dot}", "y_{dot}", "z_{dot}"];
xy_label = ["x","y"];
xy_dot_label = ["x_{dot}", "y_{dot}"];
rf_label = ["trq_{x}", "trq_{y}", "trq_{z}", "f_{x}", "f_{y}", "f_{z}"];
fb_qddot_label = ["x_{ddot}", "y_{ddot}", "z_{ddot}", "wx_{dot}", "wy_{dot}", "wz_{dot}"];

draco_joint_label = ["l\_hip\_ie", "l\_hip\_aa", "l\_hip\_fe", "l\_knee\_fe\_jp", "l\_knee\_fe\_jd", "l\_ankle\_fe", "l\_ankle\_ie", "l\_shoulder\_fe", "l\_shoulder\_aa", "l\_shoulder\_ie", "l\_elbow\_fe", "l\_wrist\_ps", "l\_wrist\_pitch", "neck\_pitch",...
    "r\_hip\_ie", "r\_hip\_aa", "r\_hip\_fe", "r\_knee\_fe\_jp", "r\_knee\_fe\_jd", "r\_ankle\_fe", "r\_ankle\_ie", "r\_shoulder\_fe", "r\_shoulder\_aa", "r\_shoulder\_ie", "r\_elbow\_fe", "r\_wrist\_ps", "r\_wrist\_pitch" ];
draco_lf_label = ["l\_hip\_ie", "l\_hip\_aa", "l\_hip\_fe", "l\_knee\_fe\_jp", "l\_knee\_fe\_jd", "l\_ankle\_fe", "l\_ankle\_ie"];
draco_rf_label = ["r\_hip\_ie", "r\_hip\_aa", "r\_hip\_fe", "r\_knee\_fe\_jp", "r\_knee\_fe\_jd", "r\_ankle\_fe", "r\_ankle\_ie"];

draco_lf_idx = zeros(1, 7);
draco_rf_idx = zeros(1, 7);
for i = 1: length(draco_lf_label)
    lf_idx = find(ismember(draco_joint_label, draco_lf_label(i)));
    rf_idx = find(ismember(draco_joint_label, draco_rf_label(i)));
    draco_lf_idx(i) = lf_idx;
    draco_rf_idx(i) = rf_idx;
end

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

% reaction force cmd
figure(5)
j = 0;
k = 0;
for i = 1:12
    subplot(6,2,i);
    if mod(i,2) == 1
        j = j + 1;
        plot(time, lf_rf_cmd(j, :), 'k', 'LineWidth', 2);
        grid on
%         min_val = min([lf_rf_cmd(j, :)]);
%         max_val = max([lf_rf_cmd(j, :)]);
%         set_fig_opt(min_val, max_val)
        xlabel('time')
        ylabel(rf_label(j))
        if j == 1
            title('left foot reaction force', 'FontSize',30)
        end
    else
        k = k + 1;
        plot(time, rf_rf_cmd(k, :), 'k', 'LineWidth', 2);
        grid on
%         min_val = min([rf_rf_cmd(k, :)]);
%         max_val = max([rf_rf_cmd(k, :)]);
%         set_fig_opt(min_val, max_val)
        xlabel('time')
        ylabel(rf_label(k))
         if j == 1
            title('right foot reaction force', 'FontSize', 30)
        end
    end
end

% floating base qddot cmd
figure(6)
for i = 1:6
    subplot(6,1,i);
    plot(time, fb_qddot_cmd(i, :), 'k', 'LineWidth', 2);
    grid on
    xlabel('time')
    ylabel(fb_qddot_label(i))
    sgtitle('floating base qddot cmd', 'FontSize', 30)
end

% foot qddot cmd
figure(7)
j = 0;
k = 0;
for i = 1:14
    subplot(7,2,i);
    if mod(i,2) == 1
        j = j + 1;
        plot(time, joint_acc_cmd(draco_lf_idx(j), :), 'k', 'LineWidth', 2);
        grid on
%         min_val = min([lf_rf_cmd(j, :)]);
%         max_val = max([lf_rf_cmd(j, :)]);
%         set_fig_opt(min_val, max_val)
        xlabel('time')
        ylabel(draco_lf_label(j))
        if j == 1
            title('left foot', 'FontSize',30)
        end
    else
        k = k + 1;
        plot(time, joint_acc_cmd(draco_rf_idx(k), :), 'k', 'LineWidth', 2);
        grid on
%         min_val = min([rf_rf_cmd(k, :)]);
%         max_val = max([rf_rf_cmd(k, :)]);
%         set_fig_opt(min_val, max_val)
        xlabel('time')
        ylabel(draco_rf_label(k))
         if j == 1
            title('right foot', 'FontSize', 30)
        end
    end
end


function [] = set_fig_opt(min,max)
    set(gca, 'LineWidth', 3)
    set(gca, 'TickLabelInterpreter', 'latex')
    set(gca, 'FontSize', 30)
    set(gca, 'Color', 'white')
    ylim([min, max])
end
