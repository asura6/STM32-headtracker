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

# USER CONFIGURATION 
#dev = HID.Connect("/dev/hidraw2")   # Device path
NUM_X = 100                         # Number of concurrent points
SAMPLE_RATE = 0.01                  # Period
# Sensor resolutions
GYRO_RES = 0.01526 #deg/s
ACCEL_RES = 0.000061035 #g
MAG_RES = (2*4912e-6)/(2**16)*10**4*10**3 #mG

# Open device
dev_path = "/dev/"+subprocess.check_output(["./find_device", ""]).decode("utf-8")[0:-1]
print("Reading data from: "+dev_path)
dev = HID.Connect(dev_path)

# Initialize variables 
sensors = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
period = NUM_X*SAMPLE_RATE 
x = np.zeros(NUM_X)
y = np.zeros([9, NUM_X]) 
t0 = 0;

# Initialize figure 
fig, ax = plt.subplots(3, sharex=True) 

ax[0].set_title('Gyroscope', fontsize=16)
ax[0].set_ylabel('Angular velocity [deg/s]', fontsize=16) 
ax[0].set_facecolor('gray')
ax[1].set_title('Accelerometer', fontsize=16)
ax[1].set_ylabel('Force [g]', fontsize=16) 
ax[1].set_facecolor('gray')
ax[2].set_title('Magnetometer', fontsize=16)
ax[2].set_ylabel('Magnetic field strength [mG]', fontsize=16)
ax[2].set_xlabel('Time [s]', fontsize=16) 
# Set bg-color
light_gray = [0.9, 0.9, 0.9]
fig.patch.set_facecolor(light_gray) 
ax[0].set_facecolor(light_gray)
ax[1].set_facecolor(light_gray)
ax[2].set_facecolor(light_gray)
# Remove ticklabels
ax[2].get_xaxis().set_ticklabels([])


# Initialize nice-looking colors 
colors = plt.cm.Set1(np.linspace(0.0, 1.0, 9)) 
# Initialize legend
labels = ['x-axis', 'y-axis', 'z-axis'] 
for i in np.arange(0,9):
    ax[i//3].plot([],[],color=colors[i % 3,:], label=labels[i % 3], lw=3)
ax[0].legend(loc='upper right')
ax[1].legend(loc='upper right') 
ax[2].legend(loc='upper right') 

lines = []
for i in np.arange(0,9):
    lobj, = ax[i // 3].plot(np.nan,np.nan, lw=3, color=colors[i % 3,:])
    lines.append(lobj) 

# initialization function: plot the background of each frame
def init(): 
    ax[0].set_ylim([-32768*GYRO_RES, 32767*GYRO_RES])
    ax[1].set_ylim([-32768*ACCEL_RES, 32767*ACCEL_RES])
    ax[2].set_ylim([-32768*MAG_RES/40, 32767*MAG_RES/40])
    for line in lines:
        line.set_data([], []) 
    return tuple(lines)

# animation function.  This is called sequentially
def animate(i): 
    global x
    global y 

    if (i < 1):
        ax[0].set_xlim([0, period]) 
        ax[1].set_xlim([0, period]) 
        ax[2].set_xlim([0, period]) 
        global t0
        t0 = time() # We want the initial timestamp close

    elif (i < NUM_X):
        x[i] = time() - t0 
        parse_HID(dev) 
        gyro_value = [value * GYRO_RES for value in sensors[1:4]]
        accel_value = [value * ACCEL_RES for value in sensors[4:7]]
        mag_value = [value * MAG_RES for value in sensors[7:10]]
        y[0:3,i] = gyro_value
        y[3:6,i] = accel_value
        y[6:9,i] = mag_value
    else:
        y = np.roll(y, -1)
        x = np.roll(x, -1)
        parse_HID(dev) 
        gyro_value = [value * GYRO_RES for value in sensors[1:4]]
        accel_value = [value * ACCEL_RES for value in sensors[4:7]] 
        mag_value = [value * MAG_RES for value in sensors[7:10]]
        y[0:3,-1] = gyro_value
        y[3:6,-1] = accel_value
        y[6:9,-1] = mag_value
        t = time() - t0
        x[-1] = t
        ax[0].set_xlim([t-period, t]) 
        ax[1].set_xlim([t-period, t]) 
        ax[2].set_xlim([t-period, t]) 

    for i in np.arange(0,9):
        lines[i].set_data(x, y[i,:]) 

    return tuple(lines) 

# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
        frames=10**100, interval=0.000000001, blit=True) 

plt.show() 

