import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime
import matplotlib.dates as mdates
from collections import deque
import numpy as np
import serial
import re
import math

PORT = "COM7"
# How many sensor samples we want to store
HISTORY_SIZE = 25000
serialport = None
# Pause re-sampling the sensor and drawing for INTERVAL seconds
INTERVAL = 0.01


def get_imu_data():
    global serialport
    if not serialport:
        # open serial port
        serialport = serial.Serial(PORT, 115200, timeout=0.1)
        # check which port was really used
        print("Opened", serialport.name)
        # Flush input
        time.sleep(3)
        serialport.readline()
    # Poll the serial port
    line = str(serialport.readline(), 'utf-8')
    if not line:
        return None
    vals = line.strip().split(',')
    if len(vals) != 3:
        return None
    try:
        vals = [float(i) for i in vals]
    except ValueError:
        return None
    print(vals)
    return vals
    # print(vals)


mag_x = deque(maxlen=HISTORY_SIZE)
mag_y = deque(maxlen=HISTORY_SIZE)
mag_z = deque(maxlen=HISTORY_SIZE)
fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)


def animate(i):
    for _ in range(30):
        ret = get_imu_data()
        if not ret:
            continue
        x = ret[0]
        y = ret[1]
        z = ret[2]

        mag_x.append(x)
        mag_y.append(y)
        mag_z.append(z)
    # Clear all axis
    ax.cla()
    # Display the sub-plots
    ax.scatter(mag_x, mag_y, color='r')
    ax.scatter(mag_y, mag_z, color='g')
    ax.scatter(mag_z, mag_x, color='b')

    if len(mag_x) == HISTORY_SIZE:
        anim.event_source.stop()
    # Pause the plot for INTERVAL seconds
    plt.pause(INTERVAL)


anim = animation.FuncAnimation(fig, animate, interval=INTERVAL)
plt.show()