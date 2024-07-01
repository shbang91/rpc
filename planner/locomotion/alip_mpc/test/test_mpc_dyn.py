import os
import array
import struct
from ctypes import*
import numpy as np
import matplotlib.pyplot as plt
from plot_utils import stance_leg3D_scatter


List = ['x_c','y_c','L_xc','L_yc','stance_leg','zH','Ts','Tr','leg_width','Lx_offset','Ly_des','kx','ky','mu','ufp_wrt_st_x','ufp_wrt_st_y','ufp_wrt_com_x','ufp_wrt_com_y']

class testData(Structure):
    _fields_ = [(name, c_double) for name in List]


#open the bin fale, then read the test data. At the end 
#we have a list of np arrays. Each array correspond to a different category in the struct.
script_directory = os.path.dirname(os.path.abspath(__file__))

file_path = os.path.join(script_directory,'build', 'Update_test.bin')
file_path2 = os.path.join(script_directory,'build', 'Full_Mpc_sol.bin')

test_data_list = []

if os.path.exists(file_path):
    with open(file_path, 'rb') as file:
        allc = testData()
        while file.readinto(allc) == sizeof(allc):
            data_instance = testData()
            memmove(addressof(data_instance), addressof(allc), sizeof(allc))
            test_data_list.append(data_instance)
else:
    print(f"File '{file_path}' not found.")


data_ar = {}

for data_instance in test_data_list:
    for name in List:
        attribute_value = getattr(data_instance, name)
        
        if name not in data_ar:
            data_ar[name] = np.array([])

        data_ar[name] = np.append(data_ar[name], attribute_value)
        
#########################
###### Second file ######
#########################
with open(file_path2, "rb") as file:
    # Read vector sizes
    xlip_size = struct.unpack("Q", file.read(8))[0]
    ufp_size = struct.unpack("Q", file.read(8))[0]

    # Read vector data
    xlip_data = struct.unpack("d" * xlip_size, file.read(xlip_size * 8))
    ufp_data = struct.unpack("d" * ufp_size, file.read(ufp_size * 8))

    # Convert tuple to list
    mpc_xlip_list = list(xlip_data)
    mpc_ufp_list = list(ufp_data)

# Print the data
print("xlip_sol:", mpc_xlip_list)
print("ufp_sol:", mpc_ufp_list)



#test, should change -1 to 1 ro -1 ...
if 1 == 0:#input("1 for printing TEST DATA  "):
    print("TEST DATA")
    for name in List:
        print(name , data_ar[name])
    print("END TEST DATA")




#############
#plot manager
#############
switch = 0#input("1 for plotting over test_it\n")
switch2 = 0#input("1 for plotting over test_it vs desired \n")


iteration = np.array(range(np.size(data_ar["x_c"])))

# Boolean indexing to separate points based on stance_leg values
st_leg_1 = np.where(data_ar['stance_leg'] == 1)
st_leg_minus_1 = np.where(data_ar['stance_leg'] == -1)

###plots 
if switch == 1:
    #plot the outputs 
    #Ufp_wrt_com
    fig1 = plt.figure()
    a = fig1.add_subplot(projection='3d')
    stance_leg3D_scatter(a, data_ar["ufp_wrt_com_x"], data_ar["ufp_wrt_com_y"], iteration, st_leg_1, st_leg_minus_1, 'r', 'b')
    plt.title("Ufp_wrt_com")
    a.set_xlabel('x')
    a.set_ylabel('y')
    a.set_zlabel('iteration')

    #Position
    fig2 = plt.figure()
    b = fig2.add_subplot(projection='3d')
    stance_leg3D_scatter(b, data_ar["x_c"], data_ar["y_c"], iteration, st_leg_1, st_leg_minus_1, 'r', 'b')
    plt.title("Position")
    b.set_xlabel('x')
    b.set_ylabel('y')
    b.set_zlabel('iteration')

    #Angular Momentum
    fig3 = plt.figure()
    c = fig3.add_subplot(projection='3d')
    stance_leg3D_scatter(c, data_ar["L_xc"], data_ar["L_yc"], iteration, st_leg_1, st_leg_minus_1, 'r', 'b')
    plt.title("Angular Momentum")
    c.set_xlabel('L_x')
    c.set_ylabel('L_y')
    c.set_zlabel('iteration')

    #Ufp_wrt_st
    fig4 = plt.figure()
    d = fig4.add_subplot(projection='3d')
    stance_leg3D_scatter(d, data_ar["ufp_wrt_st_x"], data_ar["ufp_wrt_st_y"], iteration, st_leg_1, st_leg_minus_1, 'r', 'b')
    plt.title("Ufp_wrt_st")
    d.set_xlabel('x')
    d.set_ylabel('y')
    d.set_zlabel('iteration')
    plt.legend()  

#compare x with desired x  //values are taken from matlab cassie gen. they are implicit in the .so files
mass = 31.8852
g = 9.806;
x_des =[]
y_des =[]
Lx_des=[]
Ly_des=[]
#aqui tiene más sentido simpre que ponga un - en x_des lo que es euquivalente a un - en x_c, lo pondre en x_c ya que desired es hacia delante
for i in range(np.size(data_ar["x_c"])):
    l = np.sqrt(g/data_ar["zH"][i])
    x_des.append(1/(mass*data_ar["zH"][i]*l)*np.tanh(l*data_ar["Ts"][i]/2)*data_ar["Ly_des"][i])
    y_des.append(0.5*data_ar["stance_leg"][i]*data_ar["leg_width"][i])
    Lx_des.append(0.5*data_ar["stance_leg"][i]*mass*data_ar['zH'][i]*l*data_ar['leg_width'][i]*np.tanh(l*data_ar["Ts"][i]/2)+data_ar['Lx_offset'][i])
    Ly_des.append(data_ar['Ly_des'][i])

x_des= np.array(x_des)
y_des= np.array(y_des)
Lx_des= np.array(Lx_des)
Ly_des= np.array(Ly_des)

#plot manager 2
if switch2 == 1: 
    fig5 = plt.figure()
    e = fig5.add_subplot(projection='3d')
    stance_leg3D_scatter(e, data_ar["L_xc"], data_ar["L_yc"], iteration, st_leg_1, st_leg_minus_1, 'r', 'b')
    stance_leg3D_scatter(e, Lx_des, Ly_des, iteration, st_leg_1, st_leg_minus_1, 'orange', 'cyan')
    plt.title("Angular Momentum")
    e.set_xlabel('x')
    e.set_ylabel('y')
    e.set_zlabel('iteration')
    plt.legend()  

    # had to change to - data_ar
    fig6 = plt.figure()
    f = fig6.add_subplot(projection='3d')
    stance_leg3D_scatter(f, -data_ar['x_c'], -data_ar["y_c"], iteration,st_leg_minus_1 , st_leg_1, 'r', 'b')
    stance_leg3D_scatter(f, x_des, y_des, iteration, st_leg_1, st_leg_minus_1, 'orange', 'cyan')
    plt.title("Position")
    f.set_xlabel('x')
    f.set_ylabel('y')
    f.set_zlabel('iteration')
    plt.legend()  


#######################################
##### FULL MPC DATA ###################
#######################################
x_full = [data_ar['x_c'][0]]
y_full = [data_ar['y_c'][0]]
Lx_full = []
Ly_full = []
ufp_x_full = [0]
ufp_y_full = [0]
for it in range(int(len(mpc_xlip_list)/4)):
    x_full.append(mpc_xlip_list[4*it])
    y_full.append(mpc_xlip_list[4*it+1])
    Lx_full.append(mpc_xlip_list[4*it+2])
    Ly_full.append(mpc_xlip_list[4*it+3])
    ufp_x_full.append(mpc_ufp_list[2*it])
    ufp_y_full.append(mpc_ufp_list[2*it+1])


print('x_full' , x_full)
print('y_full' , y_full)
print('Lx_full', Lx_full)
print('Ly_full', Ly_full)
print('ufp_x_full' , ufp_x_full)
print('ufp_y_full', ufp_y_full)



x_full = np.array(x_full)
ufp_x_full = np.array(ufp_x_full)
y_full = np.array(y_full)
ufp_y_full = np.array(ufp_y_full)
foot_traj_x = np.cumsum(ufp_x_full)
foot_traj_y = np.cumsum(ufp_y_full)
#with this we suposse that x_full[i] is in the coordinates of ufp_x_full[i] and ufp_x_full[i] is in coordinates of ufp_x_full[i-1]

COM_traj_x = foot_traj_x+x_full
COM_traj_y = foot_traj_y+y_full
fig7 = plt.figure()
plt.plot(COM_traj_x, COM_traj_y, marker='o', linestyle='dashed')  
plt.title('COM trajectory start coordinates')
plt.xlabel('x')
plt.ylabel('y')

fig8 = plt.figure()
plt.plot(foot_traj_x, foot_traj_y, marker='o', linestyle='dashed')  
plt.title('Full Ufp')
plt.xlabel('x')
plt.ylabel('y')


fig9 = plt.figure()
plt.plot(foot_traj_x, foot_traj_y, marker='o', linestyle='dashed') 
plt.plot(COM_traj_x, COM_traj_y, marker='o', linestyle='dashed') 
plt.plot(foot_traj_x[0], foot_traj_y[0], marker='x',markersize = 15)
plt.plot(COM_traj_x[0], COM_traj_y[0], marker='o', markersize =10) 

if data_ar['stance_leg'][0] == 1:
    plt.title('Foot and COM trajectory RS solver')
else:
    plt.title('Foot and COM trajectory LS solver')
plt.legend(['Foot', 'COM', 'Foot 0', 'COM 0'])
plt.xlabel('x')
plt.ylabel('y')

#COM with respect to desired COM
x_des2 = np.array([])
y_des2 = np.array([])
Lx_des2 = np.array([])
Ly_des2 = np.array([])
for i in range(np.size(x_full)):
    l = np.sqrt(g/data_ar["zH"][0])
    x_des2 = np.append(x_des2, 1/(mass*data_ar["zH"][0]*l)*np.tanh(l*data_ar["Ts"][0]/2)*data_ar["Ly_des"][0])
    y_des2 = np.append(y_des2, -0.5*data_ar["stance_leg"][0]*pow(-1,i)*data_ar["leg_width"][0])
    Lx_des2=np.append(Lx_des2, 0.5*data_ar["stance_leg"][0]*mass*data_ar['zH'][0]*l*data_ar['leg_width'][0]*np.tanh(l*data_ar["Ts"][0]/2)+data_ar['Lx_offset'][0])
    Ly_des2 = np.append(Ly_des2, data_ar['Ly_des'][0])


desCOMtraj_x = foot_traj_x+ x_des2
desCOMtraj_y = foot_traj_y+ y_des2
fig10 = plt.figure()
plt.plot(foot_traj_x, foot_traj_y, marker='o', linestyle='dashed') 
plt.plot(COM_traj_x, COM_traj_y, marker='o', linestyle='dashed') 
plt.plot(desCOMtraj_x,desCOMtraj_y, marker='o', linestyle='dashed')
if data_ar['stance_leg'][0] == 1:
    plt.title('Foot, COM and desired COM trajectory RS solver; indata = 1')
else:
    plt.title('Foot, COM and desired COM trajectory LS solver; indata = -1')
plt.legend(['Foot', 'COM', 'Desired COM'])
plt.xlabel('x')
plt.ylabel('y')

fig11 = plt.figure()
plt.plot(x_full, y_full, marker='o', linestyle='dashed') 
plt.plot(x_des2, y_des2, marker='o', linestyle='dashed')
plt.title('COM result and COM desired')
plt.legend(['Actual', 'Desired'])
plt.xlabel('x')
plt.ylabel('y')

#####################
#Possible problem. Because the desired X pos and Y pos depend on the current stance foot coordinates. It is not possible to achieve a position goal
#As the initial instability makes that the first step not to be perfect and creates a bias in the final solutino.
#For example if you want to stay ni place you should input 0,0 as Lx and Ly however this will move the robot to a diferent position where it will stabilize.

### 3D plots

# create the figure
fig12 = plt.figure()
# plot the floor
ax = fig12.add_subplot(111,projection='3d')
xx_max = int(max(abs(data_ar['x_c'])))+1
yy_max = int(max(abs(data_ar['y_c'])))+1
xx, yy = np.meshgrid(range(-xx_max, xx_max+1), range(-yy_max, yy_max+1))
z = data_ar['kx'][0]*xx + data_ar['ky'][0]*yy 
ax.plot_surface(xx, yy, z, alpha=0.5)

# plot the trajectories
foot_traj_z = np.array(foot_traj_x) * data_ar['kx'][0] + np.array(foot_traj_y) * data_ar['ky'][0]
COM_traj_z = np.array(COM_traj_x) * data_ar['kx'][0] + np.array(COM_traj_y) * data_ar['ky'][0] + data_ar['zH'][0]

#bx = fig12.add_subplot(111, projection='3d')
ax.plot(foot_traj_x, foot_traj_y, foot_traj_z, marker='o', linestyle='dashed')
ax.plot(COM_traj_x, COM_traj_y, COM_traj_z, marker='o', linestyle='dashed')
ax.plot(foot_traj_x[0], foot_traj_y[0], foot_traj_z[0], marker='x', markersize=10)
ax.plot(COM_traj_x[0], COM_traj_y[0], COM_traj_z[0], marker='o', markersize=5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.legend(['Foot', 'COM', 'Foot 0', 'COM 0'])



plt.show()



