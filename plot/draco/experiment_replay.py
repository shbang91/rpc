import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import pickle, yaml

import meshcat
import numpy as np
import time

# Robot model libraries
from pinocchio.visualize import MeshcatVisualizer
import pinocchio as pin

# Python-Meshcat
from meshcat.animation import Animation
from plot import meshcat_utils as vis_tools

# for the animation, set if you want to use the experiment time or start animation from t=0
use_exp_time = False
b_show_footsteps = True

# variables to load/display on Meshcat
exp_time = []
phase = []
joint_positions = []
com_position_des = []
com_position = []
com_projection = []
base_joint_quat = []
lfoot_position, rfoot_position = [], []
lfoot_orientation, rfoot_orientation = [], []
lfoot_grf, rfoot_grf = [], []
icp, icp_des = [], []
cmp_des = []

# variables loaded from yaml
t_ini_footsteps_planned, t_end_footsteps_planned = [], []
rfoot_contact_pos, rfoot_contact_ori = [], []
lfoot_contact_pos, lfoot_contact_ori = [], []

# Create Robot for Meshcat Visualization
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    cwd + "/robot_model/draco/draco3_big_feet.urdf",
    cwd + "/robot_model/draco", pin.JointModelFreeFlyer())
viz = MeshcatVisualizer(model, collision_model, visual_model)
try:
    viz.initViewer(open=True)
    # viz.viewer.wait()
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)
viz.loadViewerModel(rootNodeName="draco3")
vis_q = pin.neutral(model)

# add other visualizations to viewer
com_des_viz, com_des_model = vis_tools.add_sphere(viz.viewer,
                                                  "com_des",
                                                  color=[0., 0., 1., 0.5])
com_des_viz_q = pin.neutral(com_des_model)

com_viz, com_model = vis_tools.add_sphere(viz.viewer,
                                          "com",
                                          color=[1., 0., 0., 0.5])
com_viz_q = pin.neutral(com_model)

com_proj_viz, com_proj_model = vis_tools.add_sphere(viz.viewer,
                                                    "com_proj",
                                                    color=[0., 0., 1., 0.3])
com_proj_viz_q = pin.neutral(com_proj_model)

icp_viz, icp_model = vis_tools.add_sphere(viz.viewer,
                                          "icp",
                                          color=vis_tools.violet)
icp_viz_q = pin.neutral(icp_model)

icp_des_viz, icp_des_model = vis_tools.add_sphere(viz.viewer,
                                                  "icp_des",
                                                  color=[0., 1., 0., 0.3])
icp_des_viz_q = pin.neutral(icp_des_model)

cmp_des_viz, cmp_des_model = vis_tools.add_sphere(viz.viewer,
                                                  "cmp_des",
                                                  color=[0., 0.75, 0.75, 0.3])
cmp_des_viz_q = pin.neutral(cmp_des_model)

# add arrows visualizers to viewer
arrow_viz = meshcat.Visualizer(window=viz.viewer.window)
vis_tools.add_arrow(arrow_viz, "grf_lf", color=[1, 0, 0])
vis_tools.add_arrow(arrow_viz, "grf_rf", color=[0, 0, 1])

# add planned footsteps
footsteps_viz = meshcat.Visualizer(window=viz.viewer.window)
vis_tools.add_footstep(footsteps_viz, "lf_footstep", color=[1, 0, 0])
vis_tools.add_footstep(footsteps_viz, "rf_footstep", color=[0, 0, 1])
b_footsteps_visible_on_viewer = False

# load all YAML data from dcm trajectory manager to plot planned footsteps
footstep_plans = 0
b_footsteps_available = False
path = 'experiment_data/' + str(footstep_plans) + '.yaml'
file_exists = os.path.isfile(path)
while file_exists:
    b_footsteps_available = True
    with open(path, 'r') as stream:
        try:
            cfg = yaml.load(stream, Loader=yaml.FullLoader)
            t_ini_footsteps_planned.append(
                np.array(cfg["temporal_parameters"]["initial_time"]))
            t_end_footsteps_planned.append(
                np.array(cfg["temporal_parameters"]["final_time"]))
            rfoot_contact_pos.append(
                np.array(cfg["contact"]["right_foot"]["pos"]))
            rfoot_contact_ori.append(
                np.array(cfg["contact"]["right_foot"]["ori"]))
            # assert rfoot_contact_pos.shape[0] == rfoot_contact_ori.shape[0]
            lfoot_contact_pos.append(
                np.array(cfg["contact"]["left_foot"]["pos"]))
            lfoot_contact_ori.append(
                np.array(cfg["contact"]["left_foot"]["ori"]))
            # assert lfoot_contact_pos.shape[0] == lfoot_contact_ori.shape[0]
        except yaml.YAMLError as exc:
            print(exc)

    # check if there are more steps to queue
    footstep_plans += 1
    path = 'experiment_data/' + str(footstep_plans) + '.yaml'
    file_exists = os.path.isfile(path)

# load pkl data
with open('experiment_data/pnc.pkl', 'rb') as file:
    while True:
        try:
            d = pickle.load(file)
            exp_time.append(d['time'])
            phase.append(d['phase'])
            joint_positions.append(d['kf_base_joint_pos'] +
                                   d['kf_base_joint_ori'] +
                                   d['joint_positions'])

            com_position_des.append(d['des_com_pos'])
            com_position.append(d['act_com_pos'])
            icp.append(d['est_icp'])
            icp_des.append(d['des_icp'])

            lfoot_position.append(d['lfoot_pos'])
            rfoot_position.append(d['rfoot_pos'])
            lfoot_orientation.append(d['lfoot_ori'])
            rfoot_orientation.append(d['rfoot_ori'])

            if phase[-1] == 1:  # if in initialize state, ignore GRFs
                lfoot_grf.append([float("nan")])
                rfoot_grf.append([float("nan")])
            else:
                lfoot_grf.append(d['lfoot_rf_cmd'])
                rfoot_grf.append(d['rfoot_rf_cmd'])

            cmp_des.append(d['des_cmp'])

        except EOFError:
            break

time.sleep(3)

# replay data and create animation
curr_step_plan = 0
save_freq = 50  # hertz
anim = Animation(default_framerate=800 / save_freq)
if use_exp_time:
    effective_save_freq = 1 / 800 * save_freq
    frame_index = int(np.round(exp_time[0] / effective_save_freq))
else:
    frame_index = 0
for ti in range(len(exp_time)):
    viz.display(np.array(joint_positions[ti]))

    # plot misc parameters
    com_des_viz_q[0] = com_position_des[ti][0]
    com_des_viz_q[1] = com_position_des[ti][1]
    com_des_viz_q[2] = com_position_des[ti][2]
    com_des_viz.display(com_des_viz_q)

    com_proj_viz_q[0] = com_position_des[ti][0]
    com_proj_viz_q[1] = com_position_des[ti][1]
    com_proj_viz.display(com_proj_viz_q)

    com_viz_q[0] = com_position[ti][0]
    com_viz_q[1] = com_position[ti][1]
    com_viz_q[2] = com_position[ti][2]
    com_viz.display(com_viz_q)

    icp_viz_q[0] = icp[ti][0]
    icp_viz_q[1] = icp[ti][1]
    icp_viz.display(icp_viz_q)

    icp_des_viz_q[0] = icp_des[ti][0]
    icp_des_viz_q[1] = icp_des[ti][1]
    icp_des_viz.display(icp_des_viz_q)

    cmp_des_viz_q[0] = cmp_des[ti][0]
    cmp_des_viz_q[1] = cmp_des[ti][1]
    cmp_des_viz.display(cmp_des_viz_q)

    # plot GRFs
    # if phase[ti] != 1:
    #     vis_tools.grf_display(arrow_viz["grf_lf"], lfoot_position[ti],
    #                           lfoot_orientation[ti], lfoot_grf[ti])
    #     vis_tools.grf_display(arrow_viz["grf_rf"], rfoot_position[ti],
    #                           rfoot_orientation[ti], rfoot_grf[ti])

    # make animation
    with anim.at_frame(viz.viewer, frame_index) as frame:
        if phase[ti] != 1:
            vis_tools.grf_display(frame["grf_lf"], lfoot_position[ti],
                                  lfoot_orientation[ti], lfoot_grf[ti])
            vis_tools.grf_display(frame["grf_rf"], rfoot_position[ti],
                                  rfoot_orientation[ti], rfoot_grf[ti])

        # save other visualizations at current frame
        vis_tools.display_visualizer_frames(viz, frame)  # robot
        vis_tools.display_visualizer_frames(com_proj_viz,
                                            frame)  # projected CoM
        vis_tools.display_visualizer_frames(com_des_viz,
                                            frame)  # desired CoM position
        vis_tools.display_visualizer_frames(com_viz,
                                            frame)  # actual CoM position
        vis_tools.display_visualizer_frames(icp_viz, frame)  # actual ICP
        vis_tools.display_visualizer_frames(icp_des_viz, frame)  # desired ICP

        vis_tools.display_visualizer_frames(cmp_des_viz, frame)  # desired CMP

        # show footsteps ONLY if they have already been planned
        if b_show_footsteps and b_footsteps_available:
            if t_ini_footsteps_planned[curr_step_plan] < exp_time[
                    ti] < t_end_footsteps_planned[curr_step_plan] + 0.01:
                footsteps_viz["lf_footstep"].set_property("visible", True)
                footsteps_viz["rf_footstep"].set_property("visible", True)
                vis_tools.update_footstep(footsteps_viz["lf_footstep"],
                                          lfoot_contact_pos[curr_step_plan],
                                          lfoot_contact_ori[curr_step_plan])
                vis_tools.update_footstep(footsteps_viz["rf_footstep"],
                                          rfoot_contact_pos[curr_step_plan],
                                          rfoot_contact_ori[curr_step_plan])
            else:
                footsteps_viz["lf_footstep"].set_property("visible", False)
                footsteps_viz["rf_footstep"].set_property("visible", False)

            vis_tools.update_footstep(frame["lf_footstep"],
                                      lfoot_contact_pos[curr_step_plan],
                                      lfoot_contact_ori[curr_step_plan])
            vis_tools.update_footstep(frame["rf_footstep"],
                                      rfoot_contact_pos[curr_step_plan],
                                      rfoot_contact_ori[curr_step_plan])

            # check if we need to update the index of step plans loaded
            if exp_time[ti] > t_end_footsteps_planned[curr_step_plan] and \
                    curr_step_plan < (footstep_plans - 1):
                curr_step_plan += 1

    frame_index = frame_index + 1

footsteps_viz["lf_footstep"].set_property("visible", True)
footsteps_viz["rf_footstep"].set_property("visible", True)

print("Experiment initial time: ", exp_time[0])
print("Experiment final time: ", exp_time[-1])
viz.viewer.set_animation(anim, play=False)
