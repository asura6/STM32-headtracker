import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import HID
from time import time
from time import sleep 
import subprocess 

def parse_HID(dev): 
    rep = HID.Read(dev)
    p = HID.intArray_frompointer(rep)
    i = 0
    raw = np.zeros(10)
    for value in p:
        sensors[i] = value 
        i += 1
        if i > 9: 
            return 

def rotate(yaw, pitch, roll):
    yaw *= np.pi/180
    pitch *= np.pi/180
    roll *= np.pi/180

    Rz = np.matrix([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]])
    Ry = np.matrix([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx = np.matrix([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]])
    return Rz*Ry*Rx 

# USER CONFIGURATION 
NUM_X = 200                         # Number of concurrent points
SAMPLE_RATE = 0.01                  # Period 
# Sensor resolutions
GYRO_RES = 0.01526 #deg/s
ACCEL_RES = 0.000061035 #g
MAG_RES = (2*4912e-6)/(2**16)*10**4 #G 

# Open device
dev_path = "/dev/"+subprocess.check_output(["./find_device", ""]).decode("utf-8")[0:-1]
print("Reading data from: "+dev_path)
dev = HID.Connect(dev_path)

# Initialize variables 
sensors = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
max_vals = np.array([10**-100, 10**-100, 10**-100])
min_vals = np.array([10**100, 10**100, 10**100])
period = NUM_X*SAMPLE_RATE 
y = np.ones([3, NUM_X])*np.nan 

# Initialize figure 
fig, ax = plt.subplots(1, sharex=False) 

ax.set_title('Magnetometer', fontsize=16)
ax.set_ylabel('Magnetic field strength [G]', fontsize=16)
ax.set_xlabel('Magnetic field strength [G]', fontsize=16) 
# Set bg-color
light_gray = [0.9, 0.9, 0.9]
fig.patch.set_facecolor(light_gray) 
ax.set_facecolor(light_gray) 

# Initialize nice-looking colors 
colors = plt.cm.Set1(np.linspace(0.0, 1.0, 9)) 
# Initialize legend
labels = ['xy-plane', 'yz-plane', 'zx-plane'] 
for i in np.arange(0,3):
    ax.plot([],[],'.', color=colors[i % 3,:], label=labels[i % 3])
ax.legend(loc='upper right') 

lines = []
for i in np.arange(0,3):
    lobj, = ax.plot(np.nan,np.nan,'.', color=colors[i % 3,:])
    lines.append(lobj) 

# initialization function: plot the background of each frame
def init(): 
    ax.set_ylim([-32768*MAG_RES/40, 32767*MAG_RES/40])
    ax.set_xlim([-32768*MAG_RES/40, 32767*MAG_RES/40]) 
    for line in lines:
        line.set_data([], []) 
    return tuple(lines)

# animation function.  This is called sequentially
def animate(i): 
    global y 

    if (i < NUM_X): 
        parse_HID(dev) 
        mag_value = [value * MAG_RES for value in sensors[7:10]]
        y[:,i] = mag_value
    else:
        y = np.roll(y, -1) 
        parse_HID(dev) 
        mag_value = [value * MAG_RES for value in sensors[7:10]]
        y[:,-1] = mag_value 

    # Find max and minimum values to calculate bias
    for j in np.arange(0,3):
        if sensors[7+j] > max_vals[j]:
            max_vals[j] = sensors[7+j]
        if sensors[7+j] < min_vals[j]:
            min_vals[j] = sensors[7+j] 

    lines[0].set_data(y[0,:], y[1,:]) 
    lines[1].set_data(y[1,:], y[2,:]) 
    lines[2].set_data(y[2,:], y[0,:]) 

    return tuple(lines) 

# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
        frames=10**100, interval=10**-100, blit=True) 

plt.show() 

print("Obtained max values")
print(max_vals)
print("Obtained min values")
print(min_vals)
print("Offsets to use are") 
offsets = [np.mean([max_vals[i], min_vals[i]]) for i in np.arange(0,3)]
print([int(value) for value in offsets]) 
