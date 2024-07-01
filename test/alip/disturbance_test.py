import numpy as np
import matplotlib.pyplot as plt

#plt.style.use('ggplot')
import matplotlib.image as image
from matplotlib.offsetbox import (OffsetImage, AnnotationBbox)

import os
def read_disturbance_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()  # Remove leading/trailing whitespace
            if line == '-1':
                data.append([0, 0, 0])
            else:
                parts = line.strip('[]').split()  # Split the line into parts
                a, b, c = map(int, parts)  # Convert parts to integers
                data.append([a, b, c])
    return np.array(data)

def disturbance_range_plot(file_dir,file_name):
    file_path = os.path.join(file_dir, file_name)
    print(file_path)
    result = read_disturbance_file(file_path)

    plt.figure()
    iterations = np.arange(len(result))
    a_values = result[:, 0]
    b_values = result[:, 1]
    plt.plot(iterations, a_values, label='x dir')
    plt.plot(iterations, b_values, label='y dir')
    plt.xlabel('Iteration')
    plt.ylabel('Value')
    plt.title(file_name)
    plt.grid(True)
    plt.legend()

    return result


def disturbance_diagram(mpc_data, rl_data):
    #disturbance_diagram_2(mpc_data, rl_data)    
    fig = plt.figure(figsize=(8,8))
    plt.plot(mpc_data[:,1]/39.15,mpc_data[:,0]/39.15, linestyle = ':',color = 'tab:blue') #, color='#FFA400')
    plt.plot(mpc_data[:,3]/39.15, mpc_data[:,2]/39.15, linestyle = ':', color='tab:orange')#color='#E22A32')
    plt.plot(rl_data[:,1]/39.15, rl_data[:, 0]/39.15,color = 'tab:blue')#, color='#FFA400')
    plt.plot(rl_data[:,3]/39.15, rl_data[:, 2]/39.15, color='tab:orange')
    plt.plot(0,0, linestyle = ':', label = 'MPC', color = 'black')
    plt.plot(0,0, label = 'MPC+RL', color = 'black')
    plt.xlabel('$\\frac{F_{y}}{mass}$ $(N/kg)$', fontsize = 20)
    plt.ylabel('$\\frac{F_{x}}{mass}$ $(N/kg)$',fontsize = 20)
    plt.legend(fontsize = 16)
    plt.grid(True)
    plt.xticks(fontsize=14) 
    plt.yticks(fontsize=14) 
    # Calculate arrow position in data coordinates

    y_axis = max(max(mpc_data[:, 0]/39.15), 
                 max(rl_data[:, 0]/39.15), 
                 max(mpc_data[:,2]/39.15), 
                 max(rl_data[:, 2]/39.15))   # Adjust as needed
    x_axis = max(max(mpc_data[:,1]/39.15),
                 max(mpc_data[:,3]/39.15),
                 max(rl_data[:,1]/39.15),
                 max(rl_data[:,3]/39.15))
    plt.annotate('walking direction', xy=(0.21, 0.8), xytext=(0.135, 0.7),
             arrowprops=dict(facecolor='black', arrowstyle='->'), xycoords='figure fraction')
    plt.axis('equal')


def disturbance_diagram_2(mpc_data, rl_data):
    
    fig, ax = plt.subplots()  # Create figure and axis objects

    # Plot data
    ax.plot(mpc_data[:,1]/39.15, mpc_data[:,0]/39.15, linestyle='dashdot', label='MPC')
    ax.plot(mpc_data[:,3]/39.15, mpc_data[:,2]/39.15, linestyle='dashdot', color='#777777', label='MPC+RL')
    ax.plot(rl_data[:,1]/39.15, rl_data[:, 0]/39.15, label='MPC+RL')
    ax.plot(rl_data[:,3]/39.15, rl_data[:, 2]/39.15, color='#777777')
    
    # Plot legend
    ax.plot(0, 0, linestyle='dashdot', label='MPC', color='black')
    ax.plot(0, 0, label='MPC+RL', color='black')
    ax.legend()

    # Set labels and grid
    ax.set_xlabel('$\\frac{F_{x}}{mass}$ $(ms^{-2})$')
    ax.set_ylabel('$\\frac{F_{x}}{mass}$ $(ms^{-2})$')
    ax.grid(True)
    ax.tick_params(axis='both', which='major', labelsize=14)

    # Load and plot logo
    file = 'walking_directoin.png'
    logo = image.imread(file)
    ax.imshow(logo, extent=[-10, 10, -10, 10], aspect='auto', zorder=10)  # Set zorder to make sure it's on top


if __name__ == "__main__":
    #Freq env
    """
    MID_dist = {'MPC_xpos': disturbance_range_plot('DISTURBANCE/MID', 'disturbance_MPC_mid_xpos.txt'),
                'MPC_xneg': disturbance_range_plot('DISTURBANCE/MID', 'disturbance_MPC_mid_xneg.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/MID', 'disturbance_MPC_mid_ypos.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/MID', 'disturbance_MPC_mid_yneg.txt'),
                'RL_xpos': disturbance_range_plot('DISTURBANCE/MID', 'disturbance_RL_mid_xpos.txt'),
                'RL_xneg': disturbance_range_plot('DISTURBANCE/MID', 'disturbance_RL_mid_xneg.txt'),
                'RL_ypos': disturbance_range_plot('DISTURBANCE/MID', 'disturbance_RL_mid_ypos.txt'),
                'RL_yneg': disturbance_range_plot('DISTURBANCE/MID', 'disturbance_RL_mid_yneg.txt')
                }
    """
    """
    LONG_dist = {'MPC_xpos': disturbance_range_plot('DISTURBANCE/LONG', 'disturbance_MPC_long_xpos.txt'),
                'MPC_xneg': disturbance_range_plot('DISTURBANCE/LONG', 'disturbance_MPC_long_xneg.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/LONG', 'disturbance_MPC_long_ypos.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/LONG', 'disturbance_MPC_long_yneg.txt'),
                'RL_xpos': disturbance_range_plot('DISTURBANCE/LONG', 'disturbance_RL_long_xpos.txt'),
                'RL_xneg': disturbance_range_plot('DISTURBANCE/LONG', 'disturbance_RL_long_xneg.txt'),
                'RL_ypos': disturbance_range_plot('DISTURBANCE/LONG', 'disturbance_RL_long_ypos.txt'),
                'RL_yneg': disturbance_range_plot('DISTURBANCE/LONG', 'disturbance_RL_long_yneg.txt')
                }
    """
    """
    SHORT_dist = {'MPC_xpos': disturbance_range_plot('DISTURBANCE/SHORT', 'disturbance_MPC_short_xpos.txt'),
                'MPC_xneg': disturbance_range_plot('DISTURBANCE/SHORT', 'disturbance_MPC_short_xneg.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/SHORT', 'disturbance_MPC_short_ypos.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/SHORT', 'disturbance_MPC_short_yneg.txt'),
                'RL_xpos': disturbance_range_plot('DISTURBANCE/SHORT', 'disturbance_RL_short_xpos.txt'),
                'RL_xneg': disturbance_range_plot('DISTURBANCE/SHORT', 'disturbance_RL_short_xneg.txt'),
                'RL_ypos': disturbance_range_plot('DISTURBANCE/SHORT', 'disturbance_RL_short_ypos.txt'),
                'RL_yneg': disturbance_range_plot('DISTURBANCE/SHORT', 'disturbance_RL_short_yneg.txt')
                }
    """






    MID_max_MPC = np.stack((np.array([70, 0, 120, 0]),
                            np.array([0, 100, 0, 160]),
                            np.array([-130, 0, -160, 0]),
                            np.array([0, -100, 0, -180]),
                            np.array([70, 0, 120, 0])), axis = 0)

    MID_max_RL = np.stack((np.array([160, 0, 240, 0]),
                        np.array([0, 120, 0, 170]),
                        np.array([-160, 0, -240, 0]),
                        np.array([0, -120, 0, -200]),
                        np.array([160, 0, 240, 0])), axis = 0)

    disturbance_diagram(MID_max_MPC, MID_max_RL)


    LONG_max_MPC = np.stack((np.array([10, 0, 25, 0]),
                            np.array([0, 15, 0, 25]),
                            np.array([-25, 0, -45, 0]),
                            np.array([0, -15, 0, -20]),
                            np.array([10, 0, 25, 0])), axis = 0)

    LONG_max_RL = np.stack((np.array([35, 0, 45, 0]),
                        np.array([0, 30, 0, 35]),
                        np.array([-45, 0, -55, 0]),
                        np.array([0, -20, 0, -25]),
                        np.array([35, 0, 45, 0])), axis = 0)

    disturbance_diagram(LONG_max_MPC, LONG_max_RL)


    SHORT_max_MPC = np.stack((np.array([350, 0, 400, 0]),
                            np.array([0, 800, 0, 950]),
                            np.array([-450, 0, -550, 0]),
                            np.array([0, -500, 0, -750]),
                            np.array([350, 0, 400, 0])), axis = 0)

    SHORT_max_RL = np.stack((np.array([850, 0, 1250, 0]),
                        np.array([0, 650, 0, 850]),
                        np.array([-450, 0, -550, 0]),
                        np.array([0, -800, 0, -1050]),
                        np.array([850, 0, 1250, 0])), axis = 0)

    disturbance_diagram(SHORT_max_MPC, SHORT_max_RL)


    #/home/carlos/Desktop/Austin/SeungHyeonProject/rpc/test/alip/DISTURBANCE/MID
    """
    MID_dist = {'MPC_xpos': disturbance_range_plot('DISTURBANCE/ONE_STEP/MID', 'disturbance_MPC_mid_xpos.txt'),
                'MPC_xneg': disturbance_range_plot('DISTURBANCE/ONE_STEP/MID', 'disturbance_MPC_mid_xneg.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/ONE_STEP/MID', 'disturbance_MPC_mid_ypos.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/ONE_STEP/MID', 'disturbance_MPC_mid_yneg.txt'),
                'RL_xpos': disturbance_range_plot('DISTURBANCE/ONE_STEP/MID', 'disturbance_RL_mid_xpos.txt'),
                'RL_xneg': disturbance_range_plot('DISTURBANCE/ONE_STEP/MID', 'disturbance_RL_mid_xneg.txt'),
                'RL_ypos': disturbance_range_plot('DISTURBANCE/ONE_STEP/MID', 'disturbance_RL_mid_ypos.txt'),
                'RL_yneg': disturbance_range_plot('DISTURBANCE/ONE_STEP/MID', 'disturbance_RL_mid_yneg.txt')
                }

    LONG_dist = {'MPC_xpos': disturbance_range_plot('DISTURBANCE/ONE_STEP/LONG', 'disturbance_MPC_long_xpos.txt'),
                'MPC_xneg': disturbance_range_plot('DISTURBANCE/ONE_STEP/LONG', 'disturbance_MPC_long_xneg.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/ONE_STEP/LONG', 'disturbance_MPC_long_ypos.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/ONE_STEP/LONG', 'disturbance_MPC_long_yneg.txt'),
                'RL_xpos': disturbance_range_plot('DISTURBANCE/ONE_STEP/LONG', 'disturbance_RL_long_xpos.txt'),
                'RL_xneg': disturbance_range_plot('DISTURBANCE/ONE_STEP/LONG', 'disturbance_RL_long_xneg.txt'),
                'RL_ypos': disturbance_range_plot('DISTURBANCE/ONE_STEP/LONG', 'disturbance_RL_long_ypos.txt'),
                'RL_yneg': disturbance_range_plot('DISTURBANCE/ONE_STEP/LONG', 'disturbance_RL_long_yneg.txt')
                }

    SHORT_dist = {'MPC_xpos': disturbance_range_plot('DISTURBANCE/ONE_STEP/SHORT', 'disturbance_MPC_short_xpos.txt'),
                'MPC_xneg': disturbance_range_plot('DISTURBANCE/ONE_STEP/SHORT', 'disturbance_MPC_short_xneg.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/ONE_STEP/SHORT', 'disturbance_MPC_short_ypos.txt'),
                'MPC_ypos': disturbance_range_plot('DISTURBANCE/ONE_STEP/SHORT', 'disturbance_MPC_short_yneg.txt'),
                'RL_xpos': disturbance_range_plot('DISTURBANCE/ONE_STEP/SHORT', 'disturbance_RL_short_xpos.txt'),
                'RL_xneg': disturbance_range_plot('DISTURBANCE/ONE_STEP/SHORT', 'disturbance_RL_short_xneg.txt'),
                'RL_ypos': disturbance_range_plot('DISTURBANCE/ONE_STEP/SHORT', 'disturbance_RL_short_ypos.txt'),
                'RL_yneg': disturbance_range_plot('DISTURBANCE/ONE_STEP/SHORT', 'disturbance_RL_short_yneg.txt')
                }
    """


    MID_max_MPC = np.stack((np.array([80, 0, 90, 0]),
                            np.array([0, 20, 0, 30]),
                            np.array([-110, 0, -120, 0]),
                            np.array([0, -30, 0, -40]),
                            np.array([80, 0, 90, 0])), axis = 0)

    MID_max_RL = np.stack((np.array([90, 0, 110, 0]),
                           np.array([0, 60, 0, 70]),
                           np.array([-100, 0, -110, 0]),
                           np.array([0, -50, 0, -60]),
                           np.array([90, 0, 110, 0])), axis = 0)

    disturbance_diagram(MID_max_MPC, MID_max_RL)


    LONG_max_MPC = np.stack((np.array([20, 0, 20, 0]),
                            np.array([0, 5, 0, 5]),
                            np.array([-30, 0, -40, 0]),
                            np.array([0, -5, 0, -5]),
                            np.array([20, 0, 20, 0])), axis = 0)

    LONG_max_RL = np.stack((np.array([20, 0, 30, 0]),
                        np.array([0, 15, 0, 20]),
                        np.array([-30, 0, -35, 0]),
                        np.array([0, -15, 0, -20]),
                        np.array([20, 0, 30, 0])), axis = 0)

    disturbance_diagram(LONG_max_MPC, LONG_max_RL)


    SHORT_max_MPC = np.stack((np.array([350, 0, 350, 0]),
                            np.array([0, 150, 0, 200]),
                            np.array([-650, 0, -700, 0]),
                            np.array([0, -250, 0, -300]),
                            np.array([350, 0, 400, 0])), axis = 0)

    SHORT_max_RL = np.stack((np.array([350, 0, 400, 0]),
                             np.array([0, 350, 0, 400]),
                             np.array([-700, 0, -750, 0]),
                             np.array([0, -400, 0, -450]),
                             np.array([350, 0, 400, 0])), axis = 0)

    disturbance_diagram(SHORT_max_MPC, SHORT_max_RL)



    plt.show()