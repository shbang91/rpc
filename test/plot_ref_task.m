close all;
clear all;
clc;

addpath("/tmp")

d = dir("/tmp/task_references*.mat");

[tmp i] = max([d.datenum]);
fprintf('loading %s \n', d(i).name)
load(d(i).name)

quat_label = ["q_x", "q_y", "q_z", "q_w"];
label = ["x","y","z"];

figure(1)
for i = 1:4
    subplot(4,1,i)
    plot(time, torso_quat(i,:));
    hold on
    grid on
    xlabel('time')
    ylabel(quat_label(i))
%     legend('torso desried quat')
    sgtitle('torso desired quat')
end

figure(2)
for i = 1:3
    subplot(3,1,i)
    plot(time, torso_ang_vel(i, :));
    hold on
    grid on
    xlabel('time')
    ylabel(label(i))
%     legend('torso desired ang vel')
    sgtitle('torso desired ang vel')
end

figure(3)
for i = 1:3
    subplot(3,1,i)
    plot(time, torso_ang_acc(i, :));
    hold on
    grid on
    xlabel('time')
    ylabel(label(i))
%     legend('torso desired ang acc')
    sgtitle('torso desired ang acc')
end

