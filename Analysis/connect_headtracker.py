import HID
import numpy as np
from scipy.ndimage.interpolation import shift 
import matplotlib.pyplot as plt 
import time
# Fix to get rid of faulty warnings
import warnings
warnings.filterwarnings("ignore",".*GUI is implemented.*")

#####################
### BEGIN PROGRAM ###
#####################

dev = HID.Connect("/dev/hidraw2"); 
SENSORS = np.array([0, 0, 0, 0, 0, 0, 0]) 

def parse_HID(p):
    i = 0
    for value in p:
        SENSORS[i] = value 
        i += 1
        if i > 6:
            return

def plot_accel(t, num):
    ACCEL_RES = 0.000061035
    if x < 1:
        plt.ylim([-32768*ACCEL_RES, +32768*ACCEL_RES]) 
        plt.xlim([0, 10])
        plt.xlabel("Time [s]")
        plt.ylabel("Force [g]")
    if x > 10:
        plt.xlim([t-10, t])
    plt.plot(x, SENSORS[num]*ACCEL_RES, 'b.')
    plt.pause(0.001)

def plot_gyro(t, num, i, x0, y0, sens_type, li, ax):

    if sens_type == 'gyro':
        RES = 0.01526
    if sens_type == 'accel':
        RES = 0.000061035

    if i < 100:
        y0[0,i] = SENSORS[num]*RES
        x0[0,i] = t
    else: 
        y0 = np.roll(y0, -1)
        y0[0,-1] = SENSORS[num]*RES
        x0 = np.roll(x0, -1) 
        x0[0,-1] = t
    if x < 1:
        ax.set_ylim([-32768*RES, +32767*RES]) 
        ax.set_xlim([0, 10])
        plt.xlabel("Time [s]")
        if sens_type == 'gyro':
            plt.ylabel("Angular velocity [deg/s]")
        if sens_type == 'accel':
            plt.ylabel("Force [g]")

    if x > 10:
        ax.set_xlim([t-10, t])

    li.set_xdata(x0) 
    li.set_ydata(y0) 
    fig.canvas.draw()
    plt.pause(0.001)
    return x0, y0

y0 = np.zeros([1, 100])
x0 = np.zeros([1, 100]) 
y1 = np.zeros([1, 100])
x1 = np.zeros([1, 100]) 


x = 0
t0 = time.time() 
i = 0

plt.ion() 
fig = plt.figure()
fig.canvas.draw() 

ax1 = fig.add_subplot(121)
li1, = ax1.plot(np.nan, np.nan, 'b.')
ax2 = fig.add_subplot(122)
li2, = ax2.plot(np.nan, np.nan, 'b.')



while True: 
    rep = HID.Read(dev)
    p = HID.intArray_frompointer(rep)
    parse_HID(p) 
    # PLOT 

    x0, y0 = plot_gyro(x, 4, i, x0, y0, 'accel', li1, ax1) 
    #x1, y1 = plot_gyro(x, 2, i, x0, y0, 'gyro', li2, ax2) 
    x = time.time() - t0 
    i += 1 

while True:
    plt.pause(0.05)



