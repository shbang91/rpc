close all
clear all
clc

addpath("/tmp")
% load latest .mat file created
d = dir("/tmp/tasks_references*.mat");
d_mpc = dir("/tmp/_*.mat");
[tmp i] = max([d.datenum]);
[tmp_mpc i_mpc] = max([d_mpc.datenum]);
fprintf('loading %s \n', d(i).name)
fprintf('loading %s \n', d_mpc(i_mpc).name)
load(d(i).name)
load(d_mpc(i_mpc).name)

figure(1)
label = ["x", "y", "z"];
for i = 1 : 3
    subplot(3, 1, i)
    plot(time, unfiltered_left_force_ref(3+i, :))
    hold on 
    grid on
    plot(time, left_force_ref(3+i, :)); 
    ylabel(label(i))
    legend('unfiltered', 'interpolated')
end
sgtitle('Left Foot Contact Force')

% Right forces
figure(2)
for i = 1 : 3    
    subplot(3, 1, i)
    plot(time, unfiltered_right_force_ref(3+i, :))
    hold on 
    grid on
    plot(time, right_force_ref(3+i, :)); 
    ylabel(label(i))
    legend('unfiltered', 'interpolated')
end
sgtitle('Right Foot Contact Force')


%% MPC - RPC comparison
figure(3)
for i = 1 : 3    
    subplot(3, 1, i)
    stairs(time, unfiltered_com_pos_ref(i, :))
    hold on 
    grid on
    plot(time, com_pos_ref(i, :)); 
    plot(time_mpc, r_ref(i, :))
    ylabel(label(i))
    legend('not interpolated', 'rpc reference', 'mpc reference')
end
sgtitle('Com Position')

figure(4)
for i = 1 : 3    
    subplot(3, 1, i)
    stairs(time, unfiltered_com_vel_ref(i, :))
    hold on 
    grid on
    plot(time, com_vel_ref(i, :)); 
    ylabel(label(i))
    legend('mpc', 'rpc')
end
sgtitle('Com Velocity')

figure(5)
for i = 1 : 3    
    subplot(3, 1, i)
    stairs(time, unfiltered_com_acc_ref(i, :))
    hold on 
    grid on
    plot(time, com_acc_ref(i, :)); 
    ylabel(label(i))
    legend('mpc', 'rpc')
end
sgtitle('Com Acceleration')

figure(6)
for i = 1 : 3    
    subplot(3, 1, i)
    stairs(time, unfiltered_lf_pos_ref(i, :))
    hold on 
    grid on
    plot(time, lf_pos_ref(i, :)); 
    ylabel(label(i))
    legend('mpc', 'rpc')
end
sgtitle('Left Foot Position')
figure(7)
for i = 1 : 3    
    subplot(3, 1, i)
    stairs(time, unfiltered_lf_vel_ref(i, :))
    hold on 
    grid on
    plot(time, lf_vel_ref(i, :)); 
    ylabel(label(i))
    legend('mpc', 'rpc')
end
sgtitle('Left Foot Velocity')
figure(8)
for i = 1 : 3    
    subplot(3, 1, i)
    stairs(time, unfiltered_lf_acc_ref(i, :))
    hold on 
    grid on
    plot(time, lf_acc_ref(i, :)); 
    ylabel(label(i))
    legend('mpc', 'rpc')
end
sgtitle('Left Foot Acceleration')

figure(9)
stairs(solution_time)
grid on
title('Solution Time')
fprintf('Average solution time: %s \n', sum(solution_time)/length(solution_time))