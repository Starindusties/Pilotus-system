import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from rplidarc1 import RPLidarC1

PORT = '/dev/ttyUSB0'
lidar = RPLidarC1(PORT)

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
ax.set_theta_zero_location('N')      
ax.set_theta_direction(-1)            
ax.set_ylim(0, 12000)                 
ax.set_title('RPLIDAR C1 — live scan')

scatter = ax.scatter([], [], s=3, c='red')

def update(_):
    scan = lidar.get_scan()  # список точек (quality, angle, distance)

    angles = []
    dists = []
    for point in scan:
        q, a, d = point
        if d > 0 and q > 0:
            angles.append(np.radians(a))
            dists.append(d)

    if angles:
        scatter.set_offsets(np.column_stack([angles, dists]))

    return scatter,

ani = animation.FuncAnimation(fig, update, interval=50, blit=False)

try:
    plt.show()
finally:
    lidar.stop()
    lidar.disconnect()
