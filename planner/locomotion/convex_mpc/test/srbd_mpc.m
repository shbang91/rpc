close all;
clear;
clc;

addpath("/tmp")
exp_data_location = '/tmp';

d = dir(sprintf("srbd_draco_mpc*.mat", exp_data_location));
[tmp, i] = max([d.datenum]);
fprintf('loading %s \n', d(i).name)
load(d(i).name)

num_fig = 1;
figure(num_fig)
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time, com_pos(j, :), 'r', 'LineWidth', 3);
        grid on
        min_val = min([com_pos(j,:), com_pos(j,:)]);
        max_val = max([com_pos(j,:), com_pos(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    else
        k = k + 1;
        plot(time, lin_vel(k, :), 'r', 'LineWidth', 3);
        grid on
        min_val = min([lin_vel(k,:), lin_vel(k,:)]);
        max_val = max([lin_vel(k,:), lin_vel(k,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    end
    sgtitle('com states', 'FontSize', 30)
end

num_fig = num_fig + 1;
figure(num_fig)
j = 0;
k = 0;
for i = 1:6
    subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time, euler_ang(j, :), 'r', 'LineWidth', 3);
        grid on
        min_val = min([euler_ang(j,:), euler_ang(j,:)]);
        max_val = max([euler_ang(j,:), euler_ang(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    else
        k = k + 1;
        plot(time, ang_vel(k, :), 'r', 'LineWidth', 3);
        grid on
        min_val = min([ang_vel(k,:), ang_vel(k,:)]);
        max_val = max([ang_vel(k,:), ang_vel(k,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    end
    sgtitle('ori states', 'FontSize', 30)
end

num_fig = num_fig + 1;
figure(num_fig)
j = 0;
k = 0;
p = 0;
q = 0;
% for i = 1:12
%     subplot(3,4,i);
%     if mod(i, 4) == 1
%         j = j + 1;
%         plot(time(1:end-1), force_FL(j, :), 'r', 'LineWidth', 3);
%         grid on
%         min_val = min([force_FL(j,:), force_FL(j,:)]);
%         max_val = max([force_FL(j,:), force_FL(j,:)]);
%         min_val = min_val - 0.1 * (max_val - min_val);
%         max_val = max_val + 0.1 *(max_val - min_val);
%         set_fig_opt()
%         xlabel('time')
%     elseif mod(i, 4) == 2
%         k = k + 1;
%         plot(time(1:end-1), force_FR(k, :), 'r', 'LineWidth', 3);
%         grid on
%         min_val = min([force_FR(k,:), force_FR(k,:)]);
%         max_val = max([force_FR(k,:), force_FR(k,:)]);
%         min_val = min_val - 0.1 * (max_val - min_val);
%         max_val = max_val + 0.1 *(max_val - min_val);
%         set_fig_opt()
%         xlabel('time')
%     elseif mod(i, 4) == 3
%         p = p + 1;
%         plot(time(1:end-1), force_RL(p, :), 'r', 'LineWidth', 3);
%         grid on
%         min_val = min([force_RL(p,:), force_RL(p,:)]);
%         max_val = max([force_RL(p,:), force_RL(p,:)]);
%         min_val = min_val - 0.1 * (max_val - min_val);
%         max_val = max_val + 0.1 *(max_val - min_val);
%         set_fig_opt()
%         xlabel('time')
%     else
%         q = q + 1;
%         plot(time(1:end-1), force_RR(p, :), 'r', 'LineWidth', 3);
%         grid on
%         min_val = min([force_RR(p,:), force_RR(p,:)]);
%         max_val = max([force_RR(p,:), force_RR(p,:)]);
%         min_val = min_val - 0.1 * (max_val - min_val);
%         max_val = max_val + 0.1 *(max_val - min_val);
%         set_fig_opt()
%         xlabel('time')
%     end
%     sgtitle('reaction force', 'FontSize', 30)
% end
for i = 1:12
    subplot(6,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time(1:end-1), force_LF(j, :), 'r', 'LineWidth', 3);
        grid on
        min_val = min([force_LF(j,:), force_LF(j,:)]);
        max_val = max([force_LF(j,:), force_LF(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    else
        k = k + 1;
        plot(time(1:end-1), force_RF(k, :), 'r', 'LineWidth', 3);
        grid on
        min_val = min([force_RF(k,:), force_RF(k,:)]);
        max_val = max([force_RF(k,:), force_RF(k,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    end
    sgtitle('reaction force', 'FontSize', 30)
end
function [] = set_fig_opt()
    set(gca, 'LineWidth', 3)
    set(gca, 'TickLabelInterpreter', 'latex')
    set(gca, 'FontSize', 30)
    set(gca, 'Color', 'white')
end