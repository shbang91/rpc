close all
clear all
clc

T = 1.5;
n = 20;

addpath("/tmp")
% addpath("/mnt/ext/video/draco3/draco3_CL_no_push")
% addpath("/mnt/ext/video/draco3/draco3_CL_push_front_250m")
% load latest .mat file created
d = dir("/tmp/tasks_references*.mat");
d_mpc = dir("/tmp/_*.mat");
% d = dir("/mnt/ext/video/draco3/draco3_CL_no_push/tasks_references*.mat");
% d_mpc = dir("/mnt/ext/video/draco3/draco3_CL_no_push/_*.mat");
% d = dir("/mnt/ext/video/draco3/draco3_CL_push_front_250m/tasks_references*.mat");
% d_mpc = dir("/mnt/ext/video/draco3/draco3_CL_push_front_250m/_*.mat");
[tmp i] = max([d.datenum]);
[tmp_mpc i_mpc] = max([d_mpc.datenum]);
fprintf('loading %s \n', d(i).name)
fprintf('loading %s \n', d_mpc(i_mpc).name)
load(d(i).name)
load(d_mpc(i_mpc).name)

% booleans to enable plotting
com = true;
base = false;
left_foot_pos = true;
right_foot_pos = true;
left_foot_force = false;
right_foot_force = false;
stats = true;

label = ["x", "y", "z"];
if left_foot_force
    figure(1)   
    for i = 1 : 3
        subplot(3, 1, i)
%         plot(time, unfiltered_left_force_ref(3+i, :))
        hold on 
        grid on
        plot(time, left_force_ref(3+i, :)); 
        ylabel(label(i))
        legend('wbc reference')
    end
    sgtitle('Left Foot Contact Force')
end 

if right_foot_force
    % Right forces
    figure(2)
    for i = 1 : 3    
        subplot(3, 1, i)
%         plot(time, unfiltered_right_force_ref(3+i, :))
        hold on 
        grid on
        plot(time, right_force_ref(3+i, :)); 
        ylabel(label(i))
        legend('wbc reference')
    end
    sgtitle('Right Foot Contact Force')
end


%% MPC - RPC comparison
if com
    figure(3)
    for i = 1 : 3    
        subplot(3, 1, i)
        hold on 
        grid on
        plot(time_mpc, r_ref(i,:), 'LineWidth', 5)
        plot(time, com_pos_ref(i, :), 'LineWidth', 5); 
        plot(time, actual_com_pos(i,:), 'LineWidth', 5);
        min_val = min([r_ref(i,:), com_pos_ref(i,:), actual_com_pos(i,:)]);
        max_val = max([r_ref(i,:), com_pos_ref(i,:), actual_com_pos(i,:)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 * (max_val - min_val))
        ylabel(label(i))
        if i == 1
            legend('mpc reference', 'wbc reference', 'actual')
        end
    end
    sgtitle('Com Position', 'Fontsize', 30)
    
    figure(4)
    for i = 1 : 3    
        subplot(3, 1, i)
        hold on 
        grid on
        stairs(time_mpc, rdot_ref(i, :), 'LineWidth', 5);
        plot(time, com_vel_ref(i, :), 'LineWidth', 5); 
        plot(time, actual_com_vel(i, :), 'LineWidth', 5);
        min_val = min([rdot_ref(i,:), com_vel_ref(i,:), actual_com_vel(i,:)]);
        max_val = max([rdot_ref(i,:), com_vel_ref(i,:), actual_com_vel(i,:)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 * (max_val - min_val))
        ylabel(label(i))
        if i == 1
            legend('mpc_reference', 'wbc reference', 'actual')
        end
    end
    sgtitle('Com Velocity', 'FontSize', 30)
    
%     figure(5)
%     for i = 1 : 3    
%         subplot(3, 1, i)
% %         stairs(time, unfiltered_com_acc_ref(i, :))
%         hold on 
%         grid on
%         plot(time, com_acc_ref(i, :)); 
%         ylabel(label(i))
%         legend('wbc reference')
%     end
%     sgtitle('Com Acceleration')
end

if left_foot_pos
    figure(6)
    for i = 1 : 3    
        subplot(3, 1, i)
        plot(time_mpc, (c_ref0(i,:) + c_ref1(i,:))/2, 'LineWidth', 5)
        hold on 
        grid on
        plot(time, lf_pos_ref(i, :), 'LineWidth', 5);
        plot(time, actual_lf_pos(i, :), 'LineWidth', 5)
        min_val = min([(c_ref0(i,:) + c_ref1(i,:))/2, lf_pos_ref(i,:), actual_lf_pos(i, :)]);
        max_val = max([(c_ref0(i,:) + c_ref1(i,:))/2, lf_pos_ref(i,:), actual_lf_pos(i, :)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 * (max_val - min_val))
        ylabel(label(i))
        if i == 1
            legend('mpc reference', 'wbc reference', 'actual')
        end
    end
    sgtitle('Left Foot Position', 'FontSize', 30)
%     figure(7)
%     for i = 1 : 3    
%         subplot(3, 1, i)
% %         stairs(time, unfiltered_lf_vel_ref(i, :))
%         hold on 
%         grid on
%         plot(time, lf_vel_ref(i, :)); 
%         ylabel(label(i))
%         legend('wbc reference')
%     end
%     sgtitle('Left Foot Velocity')
%     figure(8)
%     for i = 1 : 3    
%         subplot(3, 1, i)
% %         stairs(time, unfiltered_lf_acc_ref(i, :))
%         hold on 
%         grid on
%         plot(time, lf_acc_ref(i, :)); 
%         ylabel(label(i))
%         legend('wbc reference')
%     end
%     sgtitle('Left Foot Acceleration')
end

if right_foot_pos
    figure(9)
    for i = 1 : 3    
        subplot(3, 1, i)
        plot(time_mpc, (c_ref2(i, :) + c_ref3(i,:))/2, 'LineWidth', 5)
        hold on 
        grid on
        plot(time, rf_pos_ref(i, :), 'LineWidth', 5);
        plot(time, actual_rf_pos(i, :), 'LineWidth', 5);
        min_val = min([(c_ref2(i,:) + c_ref3(i,:))/2, rf_pos_ref(i,:), actual_rf_pos(i, :)]);
        max_val = max([(c_ref2(i,:) + c_ref3(i,:))/2, rf_pos_ref(i,:), actual_rf_pos(i, :)]);
        set_fig_opt(min_val - 0.1 * (max_val - min_val), max_val + 0.1 * (max_val - min_val))
        ylabel(label(i))
        if i == 1
            legend('mpc reference', 'wbc reference', 'actual')
        end
    end
    sgtitle('Right Foot Position', 'FontSize', 30)
%     figure(10)
%     for i = 1 : 3    
%         subplot(3, 1, i)
% %         stairs(time, unfiltered_rf_vel_ref(i, :))
%         hold on 
%         grid on
%         plot(time, rf_vel_ref(i, :)); 
%         ylabel(label(i))
%         legend('wbc reference')
%     end
%     sgtitle('Right Foot Velocity')
%     figure(11)
%     for i = 1 : 3    
%         subplot(3, 1, i)
% %         stairs(time, unfiltered_rf_acc_ref(i, :))
%         hold on 
%         grid on
%         plot(time, rf_acc_ref(i, :)); 
%         ylabel(label(i))
%         legend('wbc reference')
%     end
%     sgtitle('Right Foot Acceleration')
end

if stats
    d1 = designfilt('lowpassiir','FilterOrder',12, ...
                    'HalfPowerFrequency',0.15,'DesignMethod','butter');    
    solution_time_filtered = filtfilt(d1, solution_time(1:end-1));
    figure(12)
    stairs(solution_time(1:end-1))
    grid on
    hold on
    plot(solution_time_filtered, 'LineWidth', 3)
    set_fig_opt(min(solution_time(1:end-1)) - 0.1*(max(solution_time(1:end-1)) - min(solution_time(1:end-1))),...
                max(solution_time(1:end-1)) + 0.1*(max(solution_time(1:end-1)) - min(solution_time(1:end-1))))
    title('Solution Time', 'FontSize', 30)
    fprintf('Average solution time: %s \n', sum(solution_time)/length(solution_time))

    
end

if base
    label_quat = ["x", "y", "z", "w"];
    figure(13)
    for i = 1 : 4   
        subplot(4, 1, i)    
        stairs(time_mpc, o(i, :))
        hold on 
        grid on
        plot(time, torso_ori_ref(i, :)); 
        ylabel(label_quat(i))
        legend('mpc res', 'not interpolated')
    end
    sgtitle('Torso Orientation')
end

% norm_rf_force = right_force_ref(6, :) / max(right_force_ref(6, :));
% norm_rf_pos = rf_pos_ref(3, :) / max(rf_pos_ref(3, :));
% figure
% plot(time, norm_rf_force);
% hold on 
% grid on
% plot(time, norm_rf_pos)
% legend('force', 'pos')

function [] = set_fig_opt(min, max)
    set(gca, 'LineWidth', 3)
    set(gca, 'TickLabelInterpreter', 'latex')
    set(gca, 'FontSize', 30)
    set(gcf, 'Color', 'white')
    ylim([min, max])
end
