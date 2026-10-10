import serial
import time
import numpy as np
import matplotlib.pyplot as plt

PORT = '/dev/ttyUSB0'
BAUD = 460800

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(0.5)
ser.reset_input_buffer()
ser.write(bytes([0xA5, 0x20]))
print("Ответ:", ser.read(7).hex())

plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)
ax.set_ylim(0, 12000)
ax.set_title('RPLIDAR C1')
scatter = ax.scatter([], [], s=5, c='red')
fig.canvas.draw()
fig.canvas.flush_events()

print("Пошло. Ctrl+C для выхода")

scan_points = []

try:
    while True:
        data = ser.read(5)
        if len(data) != 5:
            continue

        b0, b1, b2, b3, b4 = data
        if (b0 & 0x01) != ((b0 >> 1) & 0x01):
            continue

        quality = b0 >> 2
        angle = ((b1 >> 1) | (b2 << 7)) / 64.0
        distance = (b3 | (b4 << 8)) / 4.0

        if distance > 100 and quality > 0:
            scan_points.append((angle, distance))

        if len(scan_points) >= 100:
            angles = np.radians([p[0] for p in scan_points])
            dists = [p[1] for p in scan_points]
            scatter.set_offsets(np.column_stack([angles, dists]))
            ax.set_title(f'RPLIDAR C1 — точек: {len(scan_points)}')
            fig.canvas.draw()
            fig.canvas.flush_events()
            print(f"Точек на графике: {len(scan_points)}")

except KeyboardInterrupt:
    print("\nСтоп")
finally:
    ser.write(bytes([0xA5, 0x25]))
    ser.close()
    print("Порт закрыт.")
