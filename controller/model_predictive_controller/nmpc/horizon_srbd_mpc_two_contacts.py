import os
import sys

import rospy
from rosgraph_msgs.msg import Clock
from horizon import problem, variables
from horizon.utils import utils, kin_dyn, mat_storer
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.solvers import solver
from horizon.ros.replay_trajectory import *
from ttictoc import tic, toc
# import tf
from geometry_msgs.msg import WrenchStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
import numpy as np
from scipy.spatial.transform import Rotation as R
# from matlogger2 import matlogger
import zmq
import matplotlib.pyplot as plt

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + '/build/messages')
from pnc_to_horizon_pb2 import *
from horizon_to_pnc_pb2 import *


global set_bool
global time_mpc
global footstep_list_index
global old_footstep_list_index
global marker

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

contact_sequence = {}
robot_state = {'com_pos': list(),           # com position feedback
               'com_vel': list(),           # com velocity feedback
               'base_ori': list(),          # base orientation feedback
               'base_vel': list(),          # base angular velocity feedback
               'init_com_trj': list([[], [], []]),      # initial com trajectory from dcm planner
               'init_com_vel_trj': list([[], [], []])   # initial com velocity trajectory from dcm planner
               }

def callback(contact_sequence, robot_state, msg):
    contact_sequence.clear()
    index = msg.footstep_index
    for contact in msg.contacts:
        contact_sequence[index] = {'name': contact.name,
                                   'pos': [contact.pos_x, contact.pos_y, contact.pos_z],
                                   'ori': [contact.ori_x, contact.ori_y, contact.ori_z, contact.ori_w]}
        index += 1

    global set_bool
    global footstep_list_index
    global old_footstep_list_index

    footstep_list_index = msg.footstep_index
    if footstep_list_index != old_footstep_list_index:
        set_bool = True
    else:
        set_bool = False

    old_footstep_list_index = footstep_list_index

    robot_state['com_pos'] = msg.com_pos
    robot_state['com_vel'] = msg.com_vel
    robot_state['base_ori'] = msg.base_ori
    robot_state['base_vel'] = msg.base_vel

    robot_state['init_com_trj'][0].clear()
    robot_state['init_com_trj'][1].clear()
    robot_state['init_com_trj'][2].clear()
    robot_state['init_com_vel_trj'][0].clear()
    robot_state['init_com_vel_trj'][1].clear()
    robot_state['init_com_vel_trj'][2].clear()
    for com_pos, com_vel in zip(msg.init_com_pos, msg.init_com_vel):
        robot_state['init_com_trj'][0].append(com_pos.x)
        robot_state['init_com_trj'][1].append(com_pos.y)
        robot_state['init_com_trj'][2].append(com_pos.z)
        robot_state['init_com_vel_trj'][0].append(com_vel.xdot)
        robot_state['init_com_vel_trj'][1].append(com_vel.ydot)
        robot_state['init_com_vel_trj'][2].append(com_vel.zdot)

def clock_callback(msg):
    global time_mpc
    time_mpc = msg.clock.to_sec()

def generateSolutionMessage(solution):
    res_msg = MPCResult()

    for i in range(prb.getNNodes()):
        com = res_msg.com.add()
        com.x = solution['r'][0, i]
        com.y = solution['r'][1, i]
        com.z = solution['r'][2, i]

        com_vel = res_msg.com_vel.add()
        com_vel.xdot = solution['rdot'][0, i]
        com_vel.ydot = solution['rdot'][1, i]
        com_vel.zdot = solution['rdot'][2, i]

        base_ori = res_msg.ori.add()
        base_ori.x = solution['o'][0, i]
        base_ori.y = solution['o'][1, i]
        base_ori.z = solution['o'][2, i]
        base_ori.w = solution['o'][3, i]

        base_vel = res_msg.omega.add()
        base_vel.x = solution['w'][0, i]
        base_vel.y = solution['w'][1, i]
        base_vel.z = solution['w'][2, i]

        left_foot_pos = res_msg.left_foot_pos.add()
        left_foot_vel = res_msg.left_foot_vel.add()
        for j in range(0, contact_model):
            pos = left_foot_pos.pos.add()
            pos.x = solution['c' + str(j)][0, i]
            pos.y = solution['c' + str(j)][1, i]
            pos.z = solution['c' + str(j)][2, i]
            vel = left_foot_vel.vel.add()
            vel.xdot = solution['cdot' + str(j)][0, i]
            vel.ydot = solution['cdot' + str(j)][1, i]
            vel.zdot = solution['cdot' + str(j)][2, i]

        right_foot_pos = res_msg.right_foot_pos.add()
        right_foot_vel = res_msg.right_foot_vel.add()
        for j in range(contact_model, nc):
            pos = right_foot_pos.pos.add()
            pos.x = solution['c' + str(j)][0, i]
            pos.y = solution['c' + str(j)][1, i]
            pos.z = solution['c' + str(j)][2, i]
            vel = right_foot_vel.vel.add()
            vel.xdot = solution['cdot' + str(j)][0, i]
            vel.ydot = solution['cdot' + str(j)][1, i]
            vel.zdot = solution['cdot' + str(j)][2, i]

        # add inputs to the message
        if i != prb.getNNodes() - 1:
            com_acc = res_msg.com_acc.add()
            com_acc.xddot = solution['rddot'][0, i]
            com_acc.yddot = solution['rddot'][1, i]
            com_acc.zddot = solution['rddot'][2, i]

            base_acc = res_msg.omega_dot.add()
            base_acc.x = solution['wdot'][0, i]
            base_acc.y = solution['wdot'][1, i]
            base_acc.z = solution['wdot'][2, i]

            force_left = res_msg.force_left.add()
            left_foot_acc = res_msg.left_foot_acc.add()
            for j in range(0, contact_model):
                force = force_left.force.add()
                force.link_name = foot_frames[j]
                force.f_x = solution['f' + str(j)][0, i]
                force.f_y = solution['f' + str(j)][1, i]
                force.f_z = solution['f' + str(j)][2, i]
                acc = left_foot_acc.acc.add()
                acc.xddot = solution['cddot' + str(j)][0, i]
                acc.yddot = solution['cddot' + str(j)][0, i]
                acc.zddot = solution['cddot' + str(j)][0, i]

            force_right = res_msg.force_right.add()
            right_foot_acc = res_msg.right_foot_acc.add()
            for j in range(contact_model, nc):
                force = force_right.force.add()
                force.link_name = foot_frames[j]
                force.f_x = solution['f' + str(j)][0, i]
                force.f_y = solution['f' + str(j)][1, i]
                force.f_z = solution['f' + str(j)][2, i]
                acc = right_foot_acc.acc.add()
                acc.xddot = solution['cddot' + str(j)][0, i]
                acc.yddot = solution['cddot' + str(j)][0, i]
                acc.zddot = solution['cddot' + str(j)][0, i]

    return res_msg

def fromContactSequenceToFrames(contact):
    rot = R.from_quat(contact['ori'])
    matrix_rot = rot.as_matrix()

    if contact['name'] == "l_foot_contact":
        upper_left = np.array(contact['pos']) + np.dot(matrix_rot, np.array([0.08, 0.04, 0.0]))
        # upper_right = np.array(contact['pos']) + np.dot(matrix_rot, np.array([0.08, -0.04, 0.0]))
        # lower_left = np.array(contact['pos']) + np.dot(matrix_rot, np.array([-0.08, 0.04, 0.0]))
        lower_right = np.array(contact['pos']) + np.dot(matrix_rot, np.array([-0.08, -0.04, 0.0]))
        return list(upper_left) + list(lower_right)# + list(upper_right) + list(lower_left)
    else:
        upper_right = np.array(contact['pos']) + np.dot(matrix_rot, np.array([0.08, -0.04, 0.0]))
        lower_left = np.array(contact['pos']) + np.dot(matrix_rot, np.array([-0.08, 0.04, 0.0]))
        return list(upper_right) + list(lower_left)


def fromFramesToContactSequence(frames, name):
    sum_x = 0
    sum_y = 0
    sum_z = 0
    for frame in frames:
        sum_x += frame[0]
        sum_y += frame[1]
        sum_z += frame[2]

    contact = dict()
    contact['name'] = name
    contact['pos'] = [sum_x / len(frames), sum_y / len(frames), sum_z / len(frames)]
    contact['ori'] = [0, 0, 0, 1]

    return contact

class steps_phase:
    def __init__(self, f, c_ref, cdot, r, r_ref, rdot, rdot_ref, T, nodes, number_of_legs, contact_model, max_force, max_velocity):
        self.f = f
        self.c_ref = c_ref
        self.cdot = cdot
        self.r = r
        self.r_ref = r_ref
        self.rdot = rdot
        self.rdot_ref = rdot_ref

        self.number_of_legs = number_of_legs
        self.contact_model = contact_model
        self.max_force = max_force
        self.max_velocity = max_velocity
        self.nodes = nodes

        self.nodes = nodes
        self.step_counter = 0
        self.contact_positions = list()

    def init(self):
        #JUMP
        self.jump_f_bounds = []
        for k in range(0, 7):  # 7 nodes down
            self.jump_f_bounds.append([self.max_force, self.max_force, self.max_force])
        for k in range(0, 8):  # 8 nodes jump
            self.jump_f_bounds.append([0., 0., 0.])
        for k in range(0, 7):  # 6 nodes down
            self.jump_f_bounds.append([self.max_force, self.max_force, self.max_force])

        #NO STEP
        self.f_bounds = []
        for k in range(0, self.nodes):
            self.f_bounds.append([self.max_force, self.max_force, self.max_force])

        #STEP
        self.l_f_bounds = []
        self.r_f_bounds = []
        self.l_cdot_bounds = []
        self.r_cdot_bounds = []
        for k in range(0, 2):  # 2 nodes down
            self.l_f_bounds.append([self.max_force, self.max_force, self.max_force])
            self.r_f_bounds.append([self.max_force, self.max_force, self.max_force])
            self.l_cdot_bounds.append([0., 0., 0.])
            self.r_cdot_bounds.append([0., 0., 0.])
        for k in range(0, 8):  # 8 nodes step
            self.l_f_bounds.append([0., 0., 0.])
            self.r_f_bounds.append([self.max_force, self.max_force, self.max_force])
            self.l_cdot_bounds.append([self.max_velocity, self.max_velocity, self.max_velocity])
            self.r_cdot_bounds.append([0., 0., 0.])
        for k in range(0, 2):  # 2 nodes down
            self.l_f_bounds.append([self.max_force, self.max_force, self.max_force])
            self.r_f_bounds.append([self.max_force, self.max_force, self.max_force])
            self.l_cdot_bounds.append([0., 0., 0.])
            self.r_cdot_bounds.append([0., 0., 0.])
        for k in range(0, 8):  # 8 nodes down (other step)
            self.l_f_bounds.append([self.max_force, self.max_force, self.max_force])
            self.r_f_bounds.append([0., 0., 0.])
            self.l_cdot_bounds.append([0., 0., 0.])
            self.r_cdot_bounds.append([self.max_velocity, self.max_velocity, self.max_velocity])
        # I STILL DON'T KNOW WHY BUT THESE SHOULD BE REMOVED WHEN WALKING. MAYBE IT DEPENDS ON THE rpc -> horizon COMM
        self.l_f_bounds.append([self.max_force, self.max_force, self.max_force])
        self.r_f_bounds.append([self.max_force, self.max_force, self.max_force])
        self.l_cdot_bounds.append([0., 0., 0.])
        self.r_cdot_bounds.append([0., 0., 0.])

        #COM SWING
        self.r_swing = []
        self.rdot_swing = []
        sin = 0.05 * np.sin(np.linspace(0, 2 * np.pi, self.nodes + 1))
        cos = 0.05 * (2 * np.pi / T) * np.cos(np.linspace(0, 2 * np.pi, self.nodes + 1))
        for k in range(self.nodes):
            ''' use this when setting the left foot as world '''
            # self.r_swing.append([0., -0.1 + sin[k], 0.6])
            ''' use this when using pybullet world '''
            self.r_swing.append([0.05, sin[k], 0.6])
            self.rdot_swing.append([0., cos[k], 0.])

        self.action = ""
    def setContactPositions(self, current_contacts, next_contacts):
        # current_contacts and next_contacts MUST be filled with the first left foot
        number_of_contacts = self.number_of_legs * self.contact_model
        if len(current_contacts) != 3*number_of_contacts:
            print(bcolors.FAIL + f"current_contacts must have size: {3*number_of_contacts}" + bcolors.ENDC)
            exit()
        elif len(next_contacts) != 3*number_of_contacts:
            print(bcolors.FAIL + f"next_contacts must have size: {3*number_of_contacts}" + bcolors.ENDC)
            exit()

        self.contact_positions = []
        sin = 0.05 * np.sin(np.linspace(0, np.pi, 8))
        x_lf = np.linspace(current_contacts[0::3], next_contacts[0:3*self.contact_model][0::3] + current_contacts[3*self.contact_model:][0::3], 8)
        y_lf = np.linspace(current_contacts[1::3], next_contacts[0:3*self.contact_model][1::3] + current_contacts[3*self.contact_model:][1::3], 8)
        x_rf = np.linspace(next_contacts[0:3*self.contact_model][0::3] + current_contacts[3*self.contact_model:][0::3], next_contacts[0::3], 8)
        y_rf = np.linspace(next_contacts[0:3*self.contact_model][1::3] + current_contacts[3*self.contact_model:][1::3], next_contacts[1::3], 8)

        print(y_lf)
        print(y_rf)

        for k in range(0, 2):  # 2 nodes down
            self.contact_positions.append(current_contacts)
            self.contact_positions[-1][2::3] = [0] * number_of_contacts
        for k in range(0, 8):  # 8 nodes step with left foot
            self.contact_positions.append(next_contacts[0:3*self.contact_model] + current_contacts[3*self.contact_model:])
            self.contact_positions[-1][0::3] = x_lf[k]
            self.contact_positions[-1][1::3] = y_lf[k]
            self.contact_positions[-1][2::3] = [sin[k]] * self.contact_model + [0] * self.contact_model
        for k in range(0, 2):  # 2 nodes down
            self.contact_positions.append(next_contacts[0:3*self.contact_model] + current_contacts[3*self.contact_model:])
            self.contact_positions[-1][2::3] = [0] * number_of_contacts
        for k in range(0, 8):  # 8 nodes step
            self.contact_positions.append(next_contacts[:])
            self.contact_positions[-1][0::3] = x_rf[k]
            self.contact_positions[-1][1::3] = y_rf[k]
            self.contact_positions[-1][2::3] = [0] * self.contact_model + [sin[k]] * self.contact_model
        self.contact_positions.append(next_contacts[:])

        self.stance_contact_position = []
        self.stance_contact_velocity = []
        for k in range(0, self.nodes+1):
            self.stance_contact_position.append(current_contacts)
            self.stance_contact_velocity.append([0., 0., 0.])

        self.init()

    """
    This function moves values on the left 
    """
    def moveLeftParam(self, par):
        values = par.getValues()
        for i in range(0, self.nodes):
            par.assign(values[:, i+1], nodes=i)

    def moveLeftInput(self, input):
        for i in range(0, self.nodes):
            bounds = input.getBounds(i + 1)
            input.setBounds(bounds[0], bounds[1], nodes=i)

    def moveLeftControl(self, ctrl):
        for i in range(0, self.nodes - 1):
            bounds = ctrl.getBounds(i + 1)
            ctrl.setBounds(bounds[0], bounds[1], nodes=i)

    def set(self, action):
        self.action = action
        if self.action == 'step':
            for i in range(0, contact_model):
                self.moveLeftParam(self.c_ref[i])
                self.c_ref[i].assign(self.contact_positions[0][(i * 3):(i * 3 + 3)], nodes=self.nodes)
                self.moveLeftInput(self.cdot[i])
                self.moveLeftControl(self.f[i])
                self.f[i].setBounds(-1. * np.array(self.l_f_bounds[0]), np.array(self.l_f_bounds[0]), nodes=self.nodes-1)
                self.cdot[i].setBounds(-1 * np.array(self.l_cdot_bounds[0]), np.array(self.l_cdot_bounds[0]), nodes=self.nodes)

            for i in range(contact_model, contact_model * number_of_legs):
                self.moveLeftParam(self.c_ref[i])
                self.c_ref[i].assign(self.contact_positions[0][(i * 3):(i * 3 + 3)], nodes=self.nodes)
                self.moveLeftInput(self.cdot[i])
                self.moveLeftControl(self.f[i])
                self.f[i].setBounds(-1. * np.array(self.r_f_bounds[0]), np.array(self.r_f_bounds[0]), nodes=self.nodes-1)
                self.cdot[i].setBounds(-1 * np.array(self.r_cdot_bounds[0]), np.array(self.r_cdot_bounds[0]), nodes=self.nodes)

            self.contact_positions.append(self.contact_positions[0])
            del self.contact_positions[0]
            self.r_f_bounds.append(self.r_f_bounds[0])
            self.l_f_bounds.append(self.l_f_bounds[0])
            self.r_cdot_bounds.append(self.r_cdot_bounds[0])
            self.l_cdot_bounds.append(self.l_cdot_bounds[0])
            del self.r_f_bounds[0]
            del self.l_f_bounds[0]
            del self.r_cdot_bounds[0]
            del self.l_cdot_bounds[0]

        elif self.action == 'step_left':
            for i in range(0, contact_model):
                self.moveLeftParam(self.c_ref[i])
                self.c_ref[i].assign(self.contact_positions[0][(i*3):(i * 3 + 3)], nodes=self.nodes)
                self.moveLeftControl(self.f[i])
                self.f[i].setBounds(-1 * np.array(self.l_f_bounds[0]), np.array(self.l_f_bounds[0]), nodes=self.nodes-1)

            for i in range(contact_model, contact_model * number_of_legs):
                self.moveLeftParam(self.c_ref[i])
                self.c_ref[i].assign(self.contact_positions[0][(i*3):(i*3 + 3)], nodes=self.nodes)
                self.moveLeftControl(self.f[i])
                self.f[i].setBounds(-1. * np.array(self.f_bounds[0]), np.array(self.f_bounds[0]), nodes=self.nodes)

            del self.contact_positions[0]
            self.l_f_bounds.append(self.l_f_bounds[0])
            del self.l_f_bounds[0]

        elif self.action == 'step_right':
            for i in range(0, contact_model):
                self.moveLeftParam(self.c_ref[i])
                self.c_ref[i].assign(self.contact_positions[0][(i * 3):(1 * 3 + 3)], nodes=self.nodes)
                self.moveLeftControl(f[i])
                self.f[i].setBounds(-1. * np.array(self.f_bounds[0]), np.array(self.f_bounds[0]), nodes=self.nodes)

            for i in range(contact_model, contact_model * number_of_legs):
                self.moveLeftParam(self.c_ref[i])
                self.c_ref[i].assign(self.contact_positions[0][(i * 3):(i * 3 + 3)], nodes=self.nodes)
                self.moveLeftControl(self.f[i])
                self.f[i].setBounds(-1 * np.array(self.r_f_bounds[0]), np.array(self.r_f_bounds[0]), nodes=self.nodes-1)

            del self.contact_positions[0]
            self.r_f_bounds.append(self.r_f_bounds[0])
            del self.r_f_bounds[0]

        elif self.action == 'stand':
            for i in range(0, len(c)):
                self.c_ref[i].assign(self.stance_contact_position[0][(i*3):(i*3+3)], nodes=self.nodes)
                self.moveLeftParam(self.c_ref[i])
                self.cdot[i].setBounds(-1. * np.array(self.stance_contact_velocity[0]), np.array(self.stance_contact_velocity[0]), nodes=self.nodes)
                self.moveLeftInput(self.cdot[i])
                self.f[i].setBounds(-1. * np.array(self.f_bounds[0]), np.array(self.f_bounds[0]), nodes=self.nodes-1)
                self.moveLeftControl(self.f[i])

        elif self.action == 'swing':
            self.r_ref.assign(self.r_swing[0], nodes=self.nodes)
            self.moveLeftParam(self.r_ref)
            self.rdot_ref.assign(self.rdot_swing[0], nodes=self.nodes)
            self.moveLeftParam(self.rdot_ref)
            for i in range(0, len(c)):
                self.c_ref[i].assign(self.stance_contact_position[0][(i*3):(i*3+3)], nodes=self.nodes)
                self.moveLeftParam(self.c_ref[i])
                self.cdot[i].setBounds(-1. * np.array(self.stance_contact_velocity[0]), np.array(self.stance_contact_velocity[0]), nodes=self.nodes)
                self.moveLeftInput(self.cdot[i])
                self.f[i].setBounds(-1. * np.array(self.f_bounds[0]), np.array(self.f_bounds[0]), nodes=self.nodes-1)
                self.moveLeftControl(self.f[i])

            self.r_swing.append(self.r_swing[0])
            self.rdot_swing.append(self.rdot_swing[0])
            del self.r_swing[0]
            del self.rdot_swing[0]

def publishPointTrj(points, t, name, frame, color = [0.7, 0.7, 0.7]):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = t
    marker.ns = "SRBD"
    marker.id = 1000
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    for k in range(0, points.shape[1]):
        p = Point()
        p.x = points[0, k]
        p.y = points[1, k]
        p.z = points[2, k]
        marker.points.append(p)

    marker.color.a = 1.
    marker.scale.x = 0.005
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    rospy.Publisher(name + "_trj", Marker, queue_size=10).publish(marker)

def publishFootsteps(contact_sequence):
    global marker_footstep
    for contact in contact_sequence:
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'footsteps'
        marker.id = contact + 2
        marker.action = Marker.ADD
        marker.type = Marker.CUBE
        marker.scale.x = 0.16
        marker.scale.y = 0.08
        marker.scale.z = 0.02
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 0.5
        marker.pose.position.x = contact_sequence[contact]['pos'][0]
        marker.pose.position.y = contact_sequence[contact]['pos'][1]
        marker.pose.position.z = contact_sequence[contact]['pos'][2]
        marker.pose.orientation.x = contact_sequence[contact]['ori'][0]
        marker.pose.orientation.y = contact_sequence[contact]['ori'][1]
        marker.pose.orientation.z = contact_sequence[contact]['ori'][2]
        marker.pose.orientation.w = contact_sequence[contact]['ori'][3]
        marker_footstep.markers.append(marker)

    rospy.Publisher('footsteps', MarkerArray, queue_size=10).publish(marker_footstep)

def publishInitTrj(robot_state, n_nodes):
    marker_array = MarkerArray()
    for index in range(len(robot_state['init_com_trj'][0])):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'init_com_trj'
        marker.id = index
        marker.action = Marker.ADD
        marker.type = Marker.CUBE
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.
        marker.color.g = 1.
        marker.color.b = 1.
        marker.color.a = 1.
        marker.pose.position.x = robot_state['init_com_trj'][0][index]
        marker.pose.position.y = robot_state['init_com_trj'][1][index]
        marker.pose.position.z = robot_state['init_com_trj'][2][index]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker_array.markers.append(marker)

        # marker = Marker()
        # marker.header.frame_id = 'world'
        # marker.header.stamp = rospy.Time.now()
        # marker.ns = 'init_com_vel_trj'
        # marker.id = index + n_nodes + 1
        # marker.action = Marker.ADD
        # marker.type = Marker.ARROW
        # marker.scale.x = 0.01
        # marker.scale.y = 0.01
        # marker.scale.z = 0.01
        # marker.color.r = 1.
        # marker.color.g = 0.
        # marker.color.b = 0.
        # marker.color.a = 1.
        # start_point = Point()
        # start_point.x = robot_state['init_com_trj'][0][index]
        # start_point.y = robot_state['init_com_trj'][1][index]
        # start_point.z = robot_state['init_com_trj'][2][index]
        # end_point = Point()
        # end_point.x = robot_state['init_com_vel_trj'][0][index] + start_point.x
        # end_point.y = robot_state['init_com_vel_trj'][1][index] + start_point.y
        # end_point.z = robot_state['init_com_vel_trj'][2][index] + start_point.z
        # marker.points.append(start_point)
        # marker.points.append(end_point)
        # marker_array.markers.append(marker)

    rospy.Publisher('init_com_trj', MarkerArray, queue_size=10, latch=True).publish(marker_array)

def publishContactForce(t, f, frame):
    f_msg = WrenchStamped()
    f_msg.header.stamp = t
    f_msg.header.frame_id = frame
    f_msg.wrench.force.x = f[0]
    f_msg.wrench.force.y = f[1]
    f_msg.wrench.force.z = f[2]
    f_msg.wrench.torque.x = f_msg.wrench.torque.y = f_msg.wrench.torque.z = 0.
    rospy.Publisher('f' + frame, WrenchStamped, queue_size=10).publish(f_msg)

# def SRBDTfBroadcaster(r, o, c_dict, t):
    # br = tf.TransformBroadcaster()
    # br.sendTransform(r, o, t, "SRB", "world")
    # for key, val in c_dict.items():
        # br.sendTransform(val, [0., 0., 0., 1.], t, key, "world")

# def contactTfBroadcaster(c_dict):
    # br = tf.TransformBroadcaster()
    # for key, val in c_dict.items():
        # br.sendTransform(val, [0, 0, 0, 1], rospy.Time.now(), key, 'world')

def SRBDViewer(I, base_frame, t, number_of_contacts):
    marker = Marker()
    marker.header.frame_id = base_frame
    marker.header.stamp = t
    marker.ns = "SRBD"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.
    a = I[0,0] + I[1,1] + I[2,2]
    marker.scale.x = 0.5*(I[2,2] + I[1,1])/a
    marker.scale.y = 0.5*(I[2,2] + I[0,0])/a
    marker.scale.z = 0.5*(I[0,0] + I[1,1])/a
    marker.color.a = 0.8
    marker.color.r = marker.color.g = marker.color.b = 0.7

    rospy.Publisher('box', Marker, queue_size=10).publish(marker)

    marker_array = MarkerArray()
    for i in range(0, number_of_contacts):
        m = Marker()
        m.header.frame_id = "c" + str(i)
        m.header.stamp = t
        m.ns = "SRBD"
        m.id = i + 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.
        m.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.
        m.pose.orientation.w = 1.
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.a = 0.8
        m.color.r = m.color.g = 0.0
        m.color.b = 1.0
        marker_array.markers.append(m)

    pub2 = rospy.Publisher('contacts', MarkerArray, queue_size=10).publish(marker_array)

def setWorld(frame, kindyn, q, base_link="base_link"):
    FRAME = cs.Function.deserialize(kindyn.fk(frame))
    w_p_f = FRAME(q=q)['ee_pos']
    w_r_f = FRAME(q=q)['ee_rot']
    w_T_f = np.identity(4)
    w_T_f[0:3, 0:3] = w_r_f
    w_T_f[0:3, 3] = cs.transpose(w_p_f)

    BASE_LINK = cs.Function.deserialize(kindyn.fk(base_link))
    w_p_bl = BASE_LINK(q=q)['ee_pos']
    w_r_bl = BASE_LINK(q=q)['ee_rot']
    w_T_bl = np.identity(4)
    w_T_bl[0:3, 0:3] = w_r_bl
    w_T_bl[0:3, 3] = cs.transpose(w_p_bl)

    w_T_bl_new = np.dot(np.linalg.inv(w_T_f), w_T_bl)

    rho = R.from_matrix(w_T_bl_new[0:3, 0:3]).as_quat()

    q[0:3] = w_T_bl_new[0:3, 3]
    q[3:7] = rho

rospy.init_node('srbd_mpc_test', anonymous=True)
cpp_args = list()

rospy.set_param("use_sim_time", True)

rospy.Subscriber('/clock', Clock, clock_callback)

'''
# MatLogger2
'''
# logger = matlogger.MatLogger2('/tmp/')

"""
Creates HORIZON problem. 
These parameters can not be tuned at the moment.
"""
ns = 20
prb = problem.Problem(ns, casadi_type=cs.SX)
T = 1.5

urdf_file = open(cwd + '/robot_model/draco/draco_point_contact_fb.urdf')
urdf = urdf_file.read()
urdf_file.close()

contact_model = 2
number_of_legs = 2
nc = number_of_legs * contact_model
max_iteration = 15
foot_frames = ["l_foot_contact_upper_left", "l_foot_contact_lower_right",
               "r_foot_contact_upper_right", "r_foot_contact_lower_left"]
if nc != len(foot_frames):
    print(bcolors.FAIL + 'The number of frames is different by the total number of contacts "ns"!' + bcolors.ENDC)
    exit()

joint_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
         0.0, 0.0, -0.785, 0.785, 0.785, -0.785, 0.0,   # l_hip_ie, l_hip_aa, l_hip_fe, l_knee_fe_jp, l_knee_fe_jd, l_ankle_fe, l_ankle_ie
         0.0, 0.523, 0.0, -1.57, 0.0, 0.0,              # l_shoulder_fe, l_shoulder_aa, l_shoulder_ie, l_elbow_fe, l_wrist_ps, l_wrist_pitch
         0.0,                                           # neck_pitch
         0.0, 0.0, -0.785, 0.785, 0.785, -0.785, 0.0,   # r_hip_ie, r_hip_aa, r_hip_fe, r_knee_fe_jp, r_knee_fe_jd, r_ankle_fe, r_ankle_ie
         0.0, -0.523, 0.0, -1.57, 0.0, 0.0]             # r_shoulder_fe, r_shoulder_aa, r_shoulder_ie, r_elbow_fe, r_wrist_ps, r_wrist_pitch

kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

"""
Creates problem STATE variables
"""
""" CoM Position """
r = prb.createStateVariable("r", 3)
r.setBounds([-1000, -1000, 0.55], [1000, 1000, 0.65])
r_ref = prb.createParameter("r_ref", 3)
""" Base orientation (quaternion) """
o = prb.createStateVariable("o", 4)

""" Variable to collect all position states """
q = variables.Aggregate()
q.addVariable(r)
q.addVariable(o)

""" Contacts position """
c = dict()
for i in range(0, nc):
    c[i] = prb.createStateVariable("c" + str(i), 3)  # Contact i position
    q.addVariable(c[i])

""" Contact positions references"""
c_ref = dict()
for i in range(0, nc):
    c_ref[i] = prb.createParameter("c_ref" + str(i), 3)

""" CoM Velocity and paramter to handle references """
rdot = prb.createStateVariable("rdot", 3) # CoM vel
rdot_ref = prb.createParameter('rdot_ref', 3)
rdot_ref.assign([0. ,0. , 0.], nodes=range(1, ns+1))

""" Base angular Velocity and parameter to handle references """
w = prb.createStateVariable("w", 3) # base vel
w_ref = prb.createParameter('w_ref', 3)
w_ref.assign([0., 0., 0.], nodes=range(1, ns+1))

""" Variable to collect all velocity states """
qdot = variables.Aggregate()
qdot.addVariable(rdot)
qdot.addVariable(w)

""" Contacts velocity """
cdot = dict()
cdot_ref = dict()
for i in range(0, nc):
    cdot[i] = prb.createStateVariable("cdot" + str(i), 3)  # Contact i vel
    qdot.addVariable(cdot[i])

"""
Creates problem CONTROL variables
"""
"""
Creates problem CONTROL variables: CoM acceleration and base angular accelerations
"""
rddot = prb.createInputVariable("rddot", 3) # CoM acc
wdot = prb.createInputVariable("wdot", 3) # base acc

""" Variable to collect all acceleration controls """
qddot = variables.Aggregate()
qddot.addVariable(rddot)
qddot.addVariable(wdot)

"""
Contacts acceleration and forces
"""
cddot = dict()
f = dict()
for i in range(0, nc):
    cddot[i] = prb.createInputVariable("cddot" + str(i), 3) # Contact i acc
    qddot.addVariable(cddot[i])

    f[i] = prb.createInputVariable("f" + str(i), 3) # Contact i forces

""" n
Formulate discrete time dynamics using multiple_shooting and RK2 integrator
"""
x, xdot = utils.double_integrator_with_floating_base(q.getVars(), qdot.getVars(), qddot.getVars(), base_velocity_reference_frame=cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
prb.setDynamics(xdot)
prb.setDt(T/ns)
transcription_method = 'multiple_shooting'  # can choose between 'multiple_shooting' and 'direct_collocation'
transcription_opts = dict(integrator='RK2')  # integrator used by the multiple_shooting
if transcription_method == 'direct_collocation':
    transcription_opts = dict()
th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

"""
Setting initial state, bounds and limits
"""

max_contact_force = 1500.
print(f"max_contact_force: {max_contact_force}")
i = 0
initial_foot_position = dict()
initial_foot_dict = dict()
init_pose_foot_dict = dict()
for frame in foot_frames:
    print(frame)
    FK = cs.Function.deserialize(kindyn.fk(frame))
    p = FK(q=joint_init)['ee_pos']
    print(f"{frame}: {p}")

    initial_foot_position[i] = p
    c[i].setInitialGuess(p)
    c_ref[i].assign(p, nodes=range(0, ns+1))
    f[i].setBounds([-max_contact_force, -max_contact_force, -max_contact_force], [max_contact_force, max_contact_force, max_contact_force])
    i = i + 1

"""
Add the initial contact positions to the contact_sequence
"""
FK = cs.Function.deserialize(kindyn.fk("l_foot_contact"))
p = FK(q=joint_init)['ee_pos']
p = [p.toarray()[0][0].tolist(), p.toarray()[1][0].tolist(), 0]
ori = FK(q=joint_init)['ee_rot']
rot = R.from_matrix(ori)
quat_rot = rot.as_quat()
initial_foot_dict[-2] = {'name': 'l_foot_contact', 'pos': p, 'ori': quat_rot.tolist()}
init_pose_foot_dict[0] = {'name': 'l_foot_contact', 'pos': p, 'ori': quat_rot.tolist()}

FK = cs.Function.deserialize(kindyn.fk("r_foot_contact"))
p = FK(q=joint_init)['ee_pos']
p = [p.toarray()[0][0].tolist(), p.toarray()[1][0].tolist(), 0]

ori = FK(q=joint_init)['ee_rot']
rot = R.from_matrix(ori)
quat_rot = rot.as_quat()
initial_foot_dict[-1] = {'name': 'r_foot_contact', 'pos': p, 'ori': quat_rot.tolist()}
init_pose_foot_dict[1] = {'name': 'r_foot_contact', 'pos': p, 'ori': quat_rot.tolist()}

initial_foot_dict.update(contact_sequence)
# contact_sequence = initial_foot_dict

"""
Initialize com state and com velocity
"""
COM = cs.Function.deserialize(kindyn.centerOfMass())
com = COM(q=joint_init)['com']
com[2] = 0.6
print(f"com: {com}")
r.setInitialGuess(com)
r.setBounds(com, com, 0)
r_ref.assign(com, nodes=range(0, ns+1))
rdot.setInitialGuess([0., 0., 0.])

"""
Initialize base state and base angular velocity
"""
print(f"base orientation: {joint_init[3:7]}")
o.setInitialGuess(joint_init[3:7])
o.setBounds(joint_init[3:7], joint_init[3:7], 0)
w.setInitialGuess([0., 0., 0.])
w.setBounds([0., 0., 0.], [0., 0., 0.], 0)

"""
Set up some therms of the COST FUNCTION
"""
"""
rz_tracking is used to keep the com height around the initial value
"""
rz_tracking_gain = prb.createParameter('rz_tracking_gain', 1)
rz_tracking_gain.assign(1e2)
r_tracking = prb.createParameter('r_tracking', 1)
print(f"rz_tracking_gain: {rz_tracking_gain}")
prb.createCost("rz_tracking", rz_tracking_gain * cs.sumsqr(r[2] - com[2]), nodes=range(1, ns+1))
prb.createCost("r_tracking", r_tracking * cs.sumsqr(r - r_ref), nodes=range(1, ns+1))

"""
o_tracking is used to keep the base orientation at identity, its gain is initialize at 0 and set to non-0 only when a button is pressed
"""
Wo = prb.createParameter('Wo', 1)
Wo.assign(1e2)
prb.createCost("o_tracking", Wo * cs.sumsqr(o - joint_init[3:7]), nodes=range(1, ns+1))

"""
rdot_tracking is used to track a desired velocity of the CoM
"""
rdot_tracking_gain_z = prb.createParameter('rdot_tracking_z', 1)
rdot_tracking_gain_xy = prb.createParameter('rdot_tracking_xy', 1)
rdot_tracking_gain_z.assign(1e2)
rdot_tracking_gain_xy.assign(1e1)
print(f"rdot_tracking_gain: {rdot_tracking_gain_z}")
prb.createCost("rdot_tracking_z", rdot_tracking_gain_z * cs.sumsqr(rdot[2] - rdot_ref[2]), nodes=range(1, ns+1))
prb.createCost("rdot_tracking_xy", rdot_tracking_gain_xy * cs.sumsqr(rdot[0:2] - rdot_ref[0:2]), nodes=range(1, ns+1))
# rdot.setBounds([-10., -0.1, -10.], [10, 0.1, 10], nodes=range(0, ns+1))

"""
w_tracking is used to track a desired angular velocity of the base
"""
w_tracking_gain = prb.createParameter('w_tracking_gain', 1)
w_tracking_gain.assign(1e2)
print(f"w_tracking_gain: {w_tracking_gain}")
prb.createCost("w_tracking", w_tracking_gain * cs.sumsqr(w - w_ref), nodes=range(1, ns+1))

"""
min_qddot is to minimize the acceleration control effort
"""
min_qddot_gain = prb.createParameter('min_qddot_gain', 1)
min_qddot_gain.assign(1e-3)
print(f"min_qddot_gain: {min_qddot_gain}")
prb.createCost("min_qddot", min_qddot_gain * cs.sumsqr(qddot.getVars()), nodes=list(range(0, ns)))

"""
Set up som CONSTRAINTS
"""

min_f_gain = prb.createParameter('min_f_gain', 1)
min_f_gain.assign(1e-3)
cxy_tracking_gain = prb.createParameter('cxy_tracking_gain', 1)
cz_tracking_gain = prb.createParameter('cz_tracking_gain', 1)
cz_tracking_gain.assign(1e3)
cxy_tracking_gain.assign(1e2)
print(f"min_f_gain: {min_f_gain}")
for i in range(0, nc):
    """
    min_f try to minimze the contact forces (can be seen as distribute equally the contact forces)
    """
    prb.createCost("min_f" + str(i), min_f_gain * cs.sumsqr(f[i]), nodes=list(range(0, ns)))
    """
    cz_tracking is used to track the z reference for the feet: notice that is a constraint
    """
    prb.createCost("c_tracking_xy" + str(i), cxy_tracking_gain * cs.sumsqr(c[i][0:2] - c_ref[i][0:2]))
    prb.createCost("c_tracking_z" + str(i), cz_tracking_gain * cs.sumsqr(c[i][2] - c_ref[i][2]))

""" 
Relative distance constraint
"""
fpi = []
for l in range(0, number_of_legs):
    if contact_model == 1:
        fpi.append(l)
    else:
        fpi.append(l * contact_model)
        fpi.append(l * contact_model + contact_model - 1)

#fpi = [0, 3, 4, 7] #for knagaroo expected result
#fpi = [0, 1, 2, 3] #for spot expected result

max_clearance_x = 0.4
max_clearance_y = 0.25
d_initial_1 = -(initial_foot_position[fpi[0]][0:2] - initial_foot_position[fpi[2]][0:2])
relative_pos_y_1_4 = prb.createConstraint("relative_pos_y_1_4", -c[fpi[0]][1] + c[fpi[2]][1], bounds=dict(ub= d_initial_1[1], lb=d_initial_1[1] - max_clearance_y))
# relative_pos_y_1_4 = prb.createResidual("relative_pos_y_1_4", -c[fpi[0]][1] + c[fpi[2]][1], bounds=dict(ub= d_initial_1[1], lb=d_initial_1[1] - max_clearance_y))
relative_pos_x_1_4 = prb.createConstraint("relative_pos_x_1_4", -c[fpi[0]][0] + c[fpi[2]][0], bounds=dict(ub= d_initial_1[0] + max_clearance_x, lb=d_initial_1[0] - max_clearance_x))
# relative_pos_x_1_4 = prb.createResidual("relative_pos_x_1_4", -c[fpi[0]][0] + c[fpi[2]][0], bounds=dict(ub= d_initial_1[0] + max_clearance_x, lb=d_initial_1[0] - max_clearance_x))
d_initial_2 = -(initial_foot_position[fpi[1]][0:2] - initial_foot_position[fpi[3]][0:2])
relative_pos_y_3_6 = prb.createConstraint("relative_pos_y_3_6", -c[fpi[1]][1] + c[fpi[3]][1], bounds=dict(ub= d_initial_2[1], lb=d_initial_2[1] - max_clearance_y))
relative_pos_x_3_6 = prb.createConstraint("relative_pos_x_3_6", -c[fpi[1]][0] + c[fpi[3]][0], bounds=dict(ub= d_initial_2[0] + max_clearance_x, lb=d_initial_2[0] - max_clearance_x))

"""
This constraint is used to keep points which belong to the same contacts together
note: needs as well to be rotated in future to consider w x p
TODO: use also number_of_legs
"""
w_relative = 1e3
if contact_model > 1:
    for i in range(1, contact_model):
        prb.createConstraint("relative_vel_left_" + str(i), cdot[0][0:2] - cdot[i][0:2])
        # prb.createCost("relative_vel_left_" + str(i), w_relative * cs.sumsqr(cdot[0][0:2] - cdot[i][0:2]))
    for i in range(contact_model + 1, 2 * contact_model):
        prb.createConstraint("relative_vel_right_" + str(i), cdot[contact_model][0:2] - cdot[i][0:2])
        # prb.createCost("relative_vel_right_" + str(i), w_relative * cs.sumsqr(cdot[contact_model][0:2] - cdot[i][0:2]))

"""
Friction cones and force unilaterality constraint
TODO: for now flat terrain is assumed (StanceR needs tio be used more or less everywhere for contacts)
"""
mu = 0.5
print(f"mu: {mu}")
for i, fi in f.items():
    # FRICTION CONE
    StanceR = np.identity(3, dtype=float)  # environment rotation wrt inertial frame
    fc, fc_lb, fc_ub = kin_dyn.linearized_friction_cone(fi, mu, StanceR)
    prb.createIntermediateConstraint(f"f{i}_friction_cone", fc, bounds=dict(lb=fc_lb, ub=fc_ub))

"""
Single Rigid Body Dynamics constraint: data are taken from the loaded urdf model in nominal configuration
        m(rddot - g) - sum(f) = 0
        Iwdot + w x Iw - sum(r - p) x f = 0
"""
M = cs.Function.deserialize(kindyn.crba())
m = M(q=joint_init)['B'][0, 0]
print(f"mass: {m}")
I = M(q=joint_init)['B'][3:6, 3:6]
print(f"I centroidal: {I}")
w_R_b = utils.toRot(o)

SRBD = kin_dyn.SRBD(m, w_R_b * I * w_R_b.T, f, r, rddot, c, w, wdot)
prb.createConstraint("SRBD", SRBD, bounds=dict(lb=np.zeros(6), ub=np.zeros(6)), nodes=list(range(0, ns)))

"""
online_solver
"""
hz = int(ns/T)
print(f"hz: {hz}")
rate = rospy.Rate(hz)  # 10hz

solution_time_pub = rospy.Publisher("solution_time", Float32, queue_size=10)
srbd_pub = rospy.Publisher("srbd_constraint", WrenchStamped, queue_size=10)
srbd_msg = WrenchStamped()

opts = {
        #'ipopt.adaptive_mu_globalization': 'never-monotone-mode',
        #'ipopt.mu_allow_fast_monotone_decrease': 'no',
        #'ipopt.mu_linear_decrease_factor': 0.1,
        #'ipopt.max_cpu_time': 3e-2,
        #'ipopt.hessian_approximation': 'limited-memory',
        #'ipopt.hessian_approximation_space': 'all-variables',
        #'ipopt.limited_memory_aug_solver': 'extended',
        #'ipopt.linear_system_scaling': 'slack-based',
        #'ipopt.ma27_ignore_singularity': 'yes',
        #'ipopt.ma27_skip_inertia_check': 'yes',
        #'ipopt.hessian_constant': 'yes',
        #'ipopt.jac_c_constant' : 'yes',
        #'ipopt.nlp_scaling_method': 'none',
        #'ipopt.magic_steps': 'yes',
        'ipopt.accept_every_trial_step': 'yes',
        'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': max_iteration,
        'ipopt.linear_solver': 'ma27',
        #'ipopt.warm_start_entire_iterate': 'yes',
        #'ipopt.warm_start_same_structure': 'yes',
        'ipopt.warm_start_init_point': 'yes',
        'ipopt.fast_step_computation': 'yes',
        'ipopt.print_level': 0,
        'ipopt.suppress_all_output': 'yes',
        'ipopt.sb': 'yes',
        'print_time': 0
}

solver = solver.Solver.make_solver('ipopt', prb, opts)

r.setBounds([com[0], com[1], com[2]], [com[0], com[1], com[2]], 0)
rdot.setBounds([0., 0., 0.], [0., 0., 0.], 0)
o.setBounds(joint_init[3:7], joint_init[3:7], 0)
w.setBounds([0., 0., 0.], [0., 0., 0.], 0)

solver.solve()
solution = solver.getSolutionDict()
solution['r'][:, 1] = [com[0], com[1], com[2]]
solution['rdot'][:, 1] = [0., 0., 0.]
solution['o'][:, 1] = joint_init[3:7]
solution['w'][:, 1] = [0., 0., 0.]

robot_state['com_pos'] = solution['r'][:, 1]
robot_state['com_vel'] = solution['rdot'][:, 1]

"""
Walking patter generator and scheduler
"""
wpg = steps_phase(f, c_ref, cdot, r, r_ref, rdot, rdot_ref, T, ns, number_of_legs=number_of_legs, contact_model=contact_model, max_force=max_contact_force, max_velocity=5.0)
index = 0
global set_bool
set_bool = False
first = False
index_reset = 0

global old_footstep_list_index
old_footstep_list_index = 0

global marker_footstep
marker_footstep = MarkerArray()

"""
Initialize socket
"""
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt_string(zmq.SUBSCRIBE, "")
socket.setsockopt(zmq.CONFLATE, 1)
socket.connect("tcp://127.0.0.2:5557")

context_pub = zmq.Context()
socket_pub = context_pub.socket(zmq.PUB)
socket_pub.bind("tcp://127.0.0.3:5557")

msg = DracoState()

# plt.ion()
# figure, ax = plt.subplots()
# line1 = ax.plot(solution['o'][0, :])
# line2 = ax.plot(solution['o'][1, :])
# line3 = ax.plot(solution['o'][2, :])
# line4 = ax.plot(solution['o'][3, :])

while not rospy.is_shutdown():

    # if len(contact_sequence) != 2:
    try:
        encoded_msg = socket.recv(flags=zmq.NOBLOCK)
        msg.ParseFromString(encoded_msg)
        callback(contact_sequence, robot_state, msg)
    except zmq.Again as e:
        print(bcolors.WARNING + 'No message received yet...' + bcolors.ENDC)

    if not contact_sequence:
        rate.sleep()
        continue

    """
    Set previous first element solution as bound for the variables to guarantee continuity
    """
    # open loop
    # r.setBounds(solution['r'][:, 1], solution['r'][:, 1], 0)
    # rdot.setBounds(solution['rdot'][:, 1], solution['rdot'][:, 1], 0)
    # o.setBounds(solution['o'][:, 1], solution['o'][:, 1], 0)
    # w.setBounds(solution['w'][:, 1], solution['w'][:, 1], 0)
    # closed loop
    r.setBounds(robot_state['com_pos'], robot_state['com_pos'], 0)
    rdot.setBounds(robot_state['com_vel'], robot_state['com_vel'], 0)
    o.setBounds(robot_state['base_ori'], robot_state['base_ori'], 0)
    w.setBounds(robot_state['base_vel'], robot_state['base_vel'], 0)

    for i in range(0, nc):
        c[i].setBounds(solution['c' + str(i)][:, 1], solution['c' + str(i)][:, 1], 0)
        cdot[i].setBounds(solution['cdot' + str(i)][:, 1], solution['cdot' + str(i)][:, 1], 0)

    if False: # len(robot_state['init_com_trj'][0]) > 0:
        r.setInitialGuess(robot_state['init_com_trj'])
        rdot.setInitialGuess(robot_state['init_com_vel_trj'])
        # r_ref.assign(robot_state['init_com_trj'])
        # rdot_ref.assign(robot_state['init_com_vel_trj'])
        # o.setInitialGuess([[0]*(ns+1), [0]*(ns+1), [0]*(ns+1), [1]*(ns+1)])
        # w.setInitialGuess([[0] * (ns + 1), [0] * (ns + 1), [0] * (ns + 1)])
        o.setInitialGuess(solution['o'])
        w.setInitialGuess(solution['w'])
        publishInitTrj(robot_state, ns)
    else:
        r.setInitialGuess(solution['r'])
        rdot.setInitialGuess(solution['rdot'])
        o.setInitialGuess(solution['o'])
        w.setInitialGuess(solution['w'])

    for i in range(0, nc):
        c[i].setInitialGuess(solution['c' + str(i)])
        cdot[i].setInitialGuess(solution['cdot' + str(i)])

    if len(contact_sequence) == 2:
        init_pose_foot_dict[0] = contact_sequence[list(contact_sequence)[0]]
        init_pose_foot_dict[1] = contact_sequence[list(contact_sequence)[1]]
        current_positions = fromContactSequenceToFrames(contact_sequence[list(contact_sequence)[0]]) + \
                                                        fromContactSequenceToFrames(contact_sequence[list(contact_sequence)[1]])

        if set_bool or index == 0:
            wpg.setContactPositions(current_positions, current_positions)
        if False: # index > 15:
            if not first:
                first = True
            # swinging
            # rz_tracking_gain.assign(0)
            # r_tracking.assign(1e3)          # com position tracking
            # rdot_tracking_gain.assign(1e2)  # com velocity tracking
            # Wo.assign(1e3)                  # base orientation tracking
            # min_f_gain.assign(1e-6)         # forces minimization
            # min_qddot_gain.assign(1e-3)
            # wpg.set('swing')

            # stepping
            r_tracking.assign(0)  # com position tracking
            rz_tracking_gain.assign(1e2) # com_z position tracking
            rdot_tracking_gain.assign(1e1)  # com velocity tracking
            Wo.assign(1e2)  # base orientation tracking
            w_tracking_gain.assign(1e2)
            min_f_gain.assign(1e-3)  # forces minimization
            c_tracking_gain.assign(1e2) # contact tracking
            wpg.set('step')
        else:
            rz_tracking_gain.assign(1e2)
            rdot_tracking_gain_z.assign(1e2)
            min_f_gain.assign(1e-3)
            wpg.set('stand')

    else:
        # rdot_ref.assign([0.4, 0.0, 0.0], nodes=range(0, ns))
        current_positions = fromContactSequenceToFrames(
            contact_sequence[list(contact_sequence)[0]]) + fromContactSequenceToFrames(
            contact_sequence[list(contact_sequence)[1]])
        next_positions = fromContactSequenceToFrames(
            contact_sequence[list(contact_sequence)[2]]) + fromContactSequenceToFrames(
            contact_sequence[list(contact_sequence)[3]])

        if set_bool:
            wpg.setContactPositions(current_positions, next_positions)

        # open loop
        r_tracking.assign(0)  # com position tracking
        rz_tracking_gain.assign(1e3)  # com_z position tracking
        rdot_tracking_gain_xy.assign(0)  # com velocity tracking
        rdot_tracking_gain_z.assign(1e3)
        Wo.assign(1e2)  # base orientation tracking
        w_tracking_gain.assign(1e2)
        min_f_gain.assign(1e-3)  # forces minimization
        cxy_tracking_gain.assign(1e2)  #contact xy tracking
        cz_tracking_gain.assign(1e3)  # contact z tracking

        # close loop
        # r_tracking.assign(0)  # com position tracking
        # rz_tracking_gain.assign(1e1)  # com_z position tracking
        # rdot_tracking_gain.assign(1e2)  # com velocity tracking
        # Wo.assign(1e1)  # base orientation tracking
        # w_tracking_gain.assign(1e1)
        # min_f_gain.assign(1e-2)  # forces minimization
        # min_qddot_gain.assign(0)
        # c_tracking_gain.assign(1e3)  # contact tracking

        # rdot_ref.assign([0.5, 0.0, 0.0])
        wpg.set('step')

    """
    Solve
    """
    tic()
    start = rospy.get_rostime()
    if not solver.solve():
        print(bcolors.FAIL + "Unable to solve!" + bcolors.ENDC)
    sol_time = toc()
    if sol_time > T/ns:
        print(bcolors.WARNING + f'Warning: solution time {sol_time} exceeded MPC dt!' + bcolors.ENDC)
    # logger.add('solution_time', sol_time)
    solution = solver.getSolutionDict()

    """
    Marker Publishers
    """
    t = rospy.Time.now()

    c0_hist = dict()
    for i in range(0, nc):
        c0_hist['c' + str(i)] = solution['c' + str(i)][:, 0]

    # SRBDTfBroadcaster(solution['r'][:, 0], solution['o'][:, 0], c0_hist, t)
    publishFootsteps(contact_sequence)
    SRBDViewer(I, "SRB", t, nc)
    publishPointTrj(solution["r"], t, "SRB", "world")
    for i in range(0, nc):
        publishContactForce(t, solution['f' + str(i)][:, 0], 'c' + str(i))

    ff = dict()
    for i in range(0, nc):
        ff[i] = solution["f" + str(i)][:, 0]
    srbd_0 = kin_dyn.SRBD(m, I, ff, solution["r"][:, 0], solution["rddot"][:, 0], c, solution["w"][:, 0],
                          solution["wdot"][:, 0])
    srbd_msg.header.stamp = t
    srbd_msg.wrench.force.x = srbd_0[0]
    srbd_msg.wrench.force.y = srbd_0[1]
    srbd_msg.wrench.force.z = srbd_0[2]
    srbd_msg.wrench.torque.x = srbd_0[3]
    srbd_msg.wrench.torque.y = srbd_0[4]
    srbd_msg.wrench.torque.z = srbd_0[5]
    srbd_pub.publish(srbd_msg)

    # ax.set_ylim(ymin=-0.5, ymax=1.0)
    # line1[0].set_ydata(solution['o'][0, :])
    # line2[0].set_ydata(solution['o'][1, :])
    # line3[0].set_ydata(solution['o'][2, :])
    # line4[0].set_ydata(solution['o'][3, :])
    # figure.canvas.draw()
    # figure.canvas.flush_events()

    # index_to_save = 1
    # global time_mpc
    # logger.add('r', solution['r'][:, index_to_save])
    # logger.add('r_ref', r_ref.getValues(index_to_save))
    # logger.add('rdot', solution['rdot'][:, index_to_save])
    # logger.add('rdot_ref', rdot_ref.getValues(index_to_save))
    # logger.add('o', solution['o'][:, index_to_save])
    # logger.add('w', solution['w'][:, index_to_save])
    # logger.add('w_ref', w_ref.getValues(index_to_save))
    # logger.add('time_mpc', time_mpc)
    # for j in range(nc):
        # logger.add('c' + str(j), solution['c' + str(j)][:, index_to_save])
        # logger.add('f' + str(j), solution['f' + str(j)][:, index_to_save])
        # logger.add('f' + str(j), f[j].getUpperBounds(index_to_save))
        # logger.add('c_ref' + str(j), c_ref[j].getValues(index_to_save))
        # logger.add('cdot' + str(j), solution['cdot' + str(j)][:, index_to_save])

    '''
    Send MPC solution to pnc
    '''
    res_msg = generateSolutionMessage(solution)

    serialized_msg = res_msg.SerializeToString()
    socket_pub.send(serialized_msg)

    if first:
        index_reset += 1
    index += 1
    rate.sleep()
