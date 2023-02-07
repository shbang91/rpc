%% Load data
close all;
clear;
clc;

colors = dictionary(1, 'red', 2, 'magenta', 3, 'blue', 4, 'cyan');

addpath("/tmp")
addpath("plot/yaml_tools")

d = dir("/tmp/draco_controller_data*.mat");
[tmp, i] = max([d.datenum]);
fprintf('loading %s \n', d(i).name)
load(d(i).name)

dd = dir("/tmp/draco_state_estimator_kf_data*.mat");
[tmp, i] = max([dd.datenum]);
fprintf('loading %s \n', dd(i).name)
load(dd(i).name, 'joint_pos_act')
load(dd(i).name, 'joint_vel_act')
load(dd(i).name, 'icp_est')

ddd = dir("/tmp/draco_icp_data*.mat");
[tmp, i] = max([ddd.datenum]);
fprintf('loading %s \n', ddd(i).name)
load(ddd(i).name, 'des_icp')
% load(ddd(i).name, 'act_icp')
load(ddd(i).name, 'local_des_icp')
load(ddd(i).name, 'local_act_icp')

disp('loading experiment_data/0.yaml')
current_step = ReadYaml('experiment_data/0.yaml');
initial_time = current_step.temporal_parameters.initial_time;
final_time = current_step.temporal_parameters.final_time;
time_step = current_step.temporal_parameters.time_step;
t_ds = current_step.temporal_parameters.t_ds;
t_ss = current_step.temporal_parameters.t_ss;
t_transfer = current_step.temporal_parameters.t_transfer;

curr_rfoot_contact_pos = cell2mat(current_step.contact.curr_right_foot.pos);
curr_rfoot_contact_ori = cell2mat(current_step.contact.curr_right_foot.ori);
curr_lfoot_contact_pos = cell2mat(current_step.contact.curr_left_foot.pos);
curr_lfoot_contact_ori = cell2mat(current_step.contact.curr_left_foot.ori);
rfoot_contact_pos = cell2mat(current_step.contact.right_foot.pos);
rfoot_contact_ori = cell2mat(current_step.contact.right_foot.ori);
lfoot_contact_pos = cell2mat(current_step.contact.left_foot.pos);
lfoot_contact_ori = cell2mat(current_step.contact.left_foot.ori);

t = cell2mat(current_step.reference.time);
dcm_pos_ref = cell2mat(current_step.reference.dcm_pos);
dcm_vel_ref = cell2mat(current_step.reference.dcm_vel);
com_pos_ref = cell2mat(current_step.reference.com_pos);
com_vel_ref = cell2mat(current_step.reference.com_vel);
vrp_ref = cell2mat(current_step.reference.vrp);

%% get data at times of inteterest
[~, initial_step_idx] = min(abs(time - initial_time));

time_at_swing = initial_time + t_transfer + 1.5 * t_ds;
time_at_touch_down = time_at_swing + t_ss;

[~, first_step_idx] = min(abs(time - time_at_touch_down));
[~, first_liftoff_idx] = min(abs(time - time_at_swing));

landing_rfoot_pos = act_rf_pos(:, first_step_idx);
landing_lfoot_pos = act_lf_pos(:, first_step_idx);
liftoff_icp_pos_act = icp_est(:, first_liftoff_idx);
liftoff_icp_pos_des = des_icp(:, first_liftoff_idx);

%% plot

offset = 0.05;
axis_tick_size = 10;
axis_label_size = 14;
axis_tick_color = '#434440';
axis_label_color = '#373834';
comref_linewidth = 2;
comref_linecolor = "#D95319";       % orange
dcmref_linewidth = 4;
dcmref_linecolor = "#EDB120";       % dark yellow
% dcmref_linecolor = "#FF5733";      % cornflowerblue

% DCM tracking 2-D plot
figure
p_com = plot(com_pos_ref(:, 1), com_pos_ref(:, 2), ...
    'Color', comref_linecolor, 'LineWidth', comref_linewidth);
hold on
p_dcm = plot(dcm_pos_ref(:, 1), dcm_pos_ref(:, 2), ...
    'Color', dcmref_linecolor, 'LineWidth', dcmref_linewidth);

grid on

% ICP during stepping time
p_icp_est =  plot(icp_est(1, initial_step_idx:first_step_idx),...
    icp_est(2, initial_step_idx:first_step_idx), 'c');
p_icp_start = scatter(icp_est(1, initial_step_idx), icp_est(2, initial_step_idx), 'ms');
p_icp_end = scatter(icp_est(1, first_step_idx), icp_est(2, first_step_idx), 'co');

% ICP at swing foot liftoff
p_icp_liftoff_act = scatter(liftoff_icp_pos_act(1), liftoff_icp_pos_act(2), 'c*');
p_icp_liftoff_des = scatter(liftoff_icp_pos_des(1), liftoff_icp_pos_des(2), 'k*');

% CMP during single support


% plot feet position at beginning of step
p_lfoot = scatter(act_lf_pos(1, initial_step_idx), ...
    act_lf_pos(2, initial_step_idx), 'rs', 'filled', 'MarkerFaceAlpha', 0.2);
p_rfoot = scatter(act_rf_pos(1, initial_step_idx), ...
    act_rf_pos(2, initial_step_idx), 'bs', 'filled','MarkerFaceAlpha', 0.2);

% plot footstep plan
p_lfoot_plan = scatter(curr_lfoot_contact_pos(1), ...
    curr_lfoot_contact_pos(2), 'rs');
p_rfoot_plan = scatter(curr_rfoot_contact_pos(1), ...
    curr_rfoot_contact_pos(2), 'bs');

% planned and actual feet position at landing


% plot feet margins
ax = gca;
plot_foot(ax, curr_lfoot_contact_pos, curr_lfoot_contact_ori, colors(1))
plot_foot(ax, curr_rfoot_contact_pos, curr_rfoot_contact_ori, colors(2))

legend([p_com, p_dcm, p_icp_est, p_icp_start, p_icp_end, ...
    p_icp_liftoff_act, p_icp_liftoff_des, ...
    p_lfoot, p_rfoot, p_lfoot_plan, p_rfoot_plan], ...
    {'CoM ref','DCM ref', 'ICP est', 'ICP start', 'ICP end', ...
    'ICP liftoff (act)', 'ICP liftoff (des)',...
    'LF act', 'RF act', 'initLF', 'initRF'})

xlabel('x (m)')
ylabel('y (m)')