close all;
clear;
clc;

addpath("/tmp")
addpath("plot")
exp_data_location = '/tmp';

d = dir(sprintf("%s/convex_mpc*.mat", exp_data_location));
[tmp, i] = max([d.datenum]);
fprintf('loading %s \n', d(i).name)
load(d(i).name)

%source functions
mfilepath = fileparts(which('plot_task'));
addpath(fullfile(erase(mfilepath, '/draco')));
helper_functions

% manipulate time matrix to vector
[row, col] = size(time);
for i = 2:col
    time(1:row, i) = time(1:row, i) + time(end, i-1);
end

time_vec = reshape(time, [row*col, 1]);  

time_rf = time(2:end, 1:col);
time_rf_vec = reshape(time_rf, [(row-1)*col, 1]);

num_fig = 1;
figure(num_fig)
j = 0;
k = 0;
for i = 1:6
    ax(i) = subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time_vec, com_pos(j, :), 'b', 'LineWidth', 3);
        grid on
        hold on
        plot(time_vec, des_com_pos(j, :), 'r', 'LineWidth', 3);
        min_val = min([com_pos(j,:), des_com_pos(j,:)]);
        max_val = max([com_pos(j,:), des_com_pos(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    else
        k = k + 1;
        plot(time_vec, com_vel(k, :), 'b', 'LineWidth', 3);
        grid on
        hold on
        plot(time_vec, des_com_vel(k, :), 'r', 'LineWidth', 3);
        min_val = min([com_vel(k,:), des_com_vel(k,:)]);
        max_val = max([com_vel(k,:), des_com_vel(k,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    end
    sgtitle('com states', 'FontSize', 30)
end
linkaxes(ax,'x')

num_fig = num_fig + 1;
figure(num_fig)
j = 0;
k = 0;
for i = 1:6
    ax(i) = subplot(3,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time_vec, euler_ang(j, :), 'b', 'LineWidth', 3);
        grid on
        hold on
        plot(time_vec, des_euler_ang(j, :), 'r', 'LineWidth', 3);
        min_val = min([euler_ang(j,:), des_euler_ang(j,:)]);
        max_val = max([euler_ang(j,:), des_euler_ang(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    else
        k = k + 1;
        plot(time_vec, ang_vel(k, :), 'b', 'LineWidth', 3);
        grid on
        hold on
        plot(time_vec, des_ang_vel(k, :), 'r', 'LineWidth', 3);
        min_val = min([ang_vel(k,:), des_ang_vel(k,:)]);
        max_val = max([ang_vel(k,:), des_ang_vel(k,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    end
    sgtitle('ori states', 'FontSize', 30)
end
linkaxes(ax,'x')

num_fig = num_fig + 1;
figure(num_fig)
j = 0;
k = 0;
for i = 1:12
    ax(i) = subplot(6,2,i);
    if mod(i, 2) == 1
        j = j + 1;
        plot(time_rf_vec, force_LF(j, :), 'r', 'LineWidth', 3);
        grid on
        min_val = min([force_LF(j,:), force_LF(j,:)]);
        max_val = max([force_LF(j,:), force_LF(j,:)]);
        min_val = min_val - 0.1 * (max_val - min_val);
        max_val = max_val + 0.1 *(max_val - min_val);
        set_fig_opt()
        xlabel('time')
    else
        k = k + 1;
        plot(time_rf_vec, force_RF(k, :), 'r', 'LineWidth', 3);
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
linkaxes(ax,'x')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% reference trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
num_fig = num_fig + 1;
figure(num_fig)
% Example number of time steps
[row, col] = size(des_base_pos);
n = col;
% 
% plot3(des_base_pos(1,:), des_base_pos(2,:), des_base_pos(3,:), 'k-', 'LineWidth',2);
% hold on
% 
% plot3(des_lf_pos(1,:), des_lf_pos(2,:), des_lf_pos(3,:), 'r-', 'LineWidth',2);
% plot3(des_rf_pos(1,:), des_rf_pos(2,:), des_rf_pos(3,:), 'b-', 'LineWidth',2);


% Define colors for plotting - this could be a predefined list or generated dynamically
colors = lines(ceil(n/10)); % 'jet' generates a colormap, which we use to get a variety of colors

% Loop through each segment of 10 steps
for i = 1:10:n-1
    % Determine the end of the current segment
    endIndex = min(i+9, n);
    
    % Extract the current segment of the trajectory
    segmentX = des_rf_pos(1, i:endIndex);
    segmentY = des_rf_pos(2, i:endIndex);
    segmentZ = des_rf_pos(3, i:endIndex);

    segmentBaseX = des_base_pos(1, i:endIndex);
    
    % Determining the color index
    colorIndex = ceil(i/10);
    if colorIndex > size(colors, 1)
        colorIndex = mode(colorIndex, size(colors, 1)) + 1;
    end

    plot(i:endIndex,segmentY, '-', 'LineWidth', 2, 'Color', colors(colorIndex, :));
    % Plot the current segment
    % Use mod or another strategy to cycle through colors if there are more segments than colors
%     plot3(segmentX, segmentY, segmentZ, '-', 'LineWidth', 2, 'Color',colors(colorIndex, :));
    hold on; % Keep the plot window open to overlay the next plots
    plot(i:endIndex, segmentBaseY, '*', 'LineWidth',2, 'Color','k');
end


% Enhancing the plot

% xlabel('X Axis');
% ylabel('Y Axis');
% zlabel('Z Axis');
% title('Robot Base Trajectory with Color Variation');
% grid on; % Adding a grid
% axis equal; % Equal scaling along each axis for proportional representation
% view(3); % Setting the view to 3D
% hold off; % Release the plot window

% function [] = set_fig_opt()
%     set(gca, 'LineWidth', 3)
%     set(gca, 'TickLabelInterpreter', 'latex')
%     set(gca, 'FontSize', 30)
%     set(gca, 'Color', 'white')
% end