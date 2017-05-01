import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import mpl_toolkits.mplot3d.axes3d as p3
import HID
from time import time
from time import sleep 
import subprocess 

def parse_HID(dev): 
    rep = HID.Read(dev)
    p = HID.intArray_frompointer(rep)
    i = 0
    raw = np.zeros(13)
    for value in p: 
        sensors[i] = value 
        i += 1
        if i > 12: 
            return 

# USER CONFIGURATION 
NUM_X = 100                         # Number of concurrent points
SAMPLE_RATE = 0.01                  # Period 

# Open device
dev_path = "/dev/"+subprocess.check_output(["./find_device", ""]).decode("utf-8")[0:-1]
print("Reading data from: "+dev_path)
dev = HID.Connect(dev_path)

# Initialize variables 
sensors = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
period = NUM_X*SAMPLE_RATE 
x = np.ones(NUM_X)*np.nan
y = np.ones([3, NUM_X])*np.nan 
t0 = 0;

fig = plt.figure()
ax = p3.Axes3D(fig)

# Create a sphere 
u = np.linspace(-0.1, 0.1, 100)
v = np.linspace(-0.1, 0.1, 100)
x = 0.05*np.outer(np.cos(u*np.pi/0.1), np.sin(v*np.pi/0.1))
y = 0.05*np.outer(np.sin(u*np.pi/0.1), np.sin(v*np.pi/0.1))
z = 0.05*np.outer(np.ones(np.size(u*np.pi/0.1)), np.cos(v*np.pi/0.1)) 

# Configure the figure cosmetically 
ax.set_title('3D-orientation', fontsize=16) 
# # Remove tick-labels
ax.set_xticklabels([]) 
ax.set_yticklabels([]) 
ax.set_zticklabels([]) 
# # Set yabels
ax.set_ylabel('Y-axis')
ax.set_xlabel('X-axis')
ax.set_zlabel('Z-axis')
# # Set ranges
ax.set_xlim3d([-1, 1])
ax.set_ylim3d([-1, 1])
ax.set_zlim3d([-1, 1])

lines = []
lobj, = ax.plot([0.5,0.5],[0.5,0.5],[0.5,0.5], color='r', label='Yaw')
lines.append(lobj)
lobj, = ax.plot([0.5,0.5],[0.5,0.5],[0.5,0.5], color='b', label='Pitch')
lines.append(lobj)
lobj, = ax.plot([0.5,0.5],[0.5,0.5],[0.5,0.5], color='g', label='Roll')
lines.append(lobj)

#plot sphere
ax.plot_surface(x, y, z, color='b') 

ax.legend(loc='upper right')

def animate(i):
    parse_HID(dev) 
    yaw = sensors[10]*((10.*np.pi)/32768.)
    pitch = sensors[11]*((10.*np.pi)/32768.)
    roll = sensors[12]*((10.*np.pi)/32768.)

    # Yaw
    lines[0].set_data([0, np.cos(yaw)],[0, np.sin(yaw)])
    lines[0].set_3d_properties([0, 0])
    # Pitch
    lines[1].set_data([0, 0],[0, 0])
    lines[1].set_3d_properties([0, np.sin(pitch)])
    # Roll
    lines[2].set_data([0, np.sin(roll)],[0, 0])
    lines[2].set_3d_properties([0, np.cos(roll)]) 

    return lines


# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, 
        frames=10**100, interval=0.000000001, blit=True) 

plt.show() 
