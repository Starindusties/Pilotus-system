import serial
import time
import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

PORT = '/dev/ttyUSB0'
BAUD = 460800
MAX_DIST = 6000

# --- общий буфер между потоками ---
scan_points = []
lock = threading.Lock()
running = True

# --- поток чтения ---
def reader_thread():
    global running
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(bytes([0xA5, 0x20]))
    print("Ответ:", ser.read(7).hex())

    while running:
        data = ser.read(5)
        if len(data) != 5:
            continue
        b0, b1, b2, b3, b4 = data
        if (b0 & 0x01) != ((b0 >> 1) & 0x01):
            continue
        quality = b0 >> 2
        angle = ((b1 >> 1) | (b2 << 7)) / 64.0
        distance = (b3 | (b4 << 8)) / 4.0
        if 100 < distance < MAX_DIST and quality > 20:
            with lock:
                scan_points.append((angle, distance))

    ser.write(bytes([0xA5, 0x25]))
    ser.close()
    print("Порт закрыт.")

# --- запускаем чтение в фоне ---
t = threading.Thread(target=reader_thread, daemon=True)
t.start()

# --- главный поток: только рисуем ---
plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)
ax.set_ylim(0, MAX_DIST)
ax.set_yticks([1000, 2000, 3000, 4000, 5000, 6000])
ax.set_title('RPLIDAR C1')
scatter = ax.scatter([], [], s=5, c='red')
fig.canvas.draw()
fig.canvas.flush_events()

print("Пошло. Ctrl+C для выхода")

frames = 0

try:
    while True:
        # раз в 1 секунду берём снимок буфера и рисуем
        time.sleep(1.0)

        with lock:
            snapshot = list(scan_points)
            scan_points.clear()

        if snapshot:
            angles = np.radians([p[0] for p in snapshot])
            dists = [p[1] for p in snapshot]
            scatter.set_offsets(np.column_stack([angles, dists]))
            frames += 1
            ax.set_title(f'RPLIDAR C1 — кадр {frames}, точек: {len(snapshot)}')
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            print(f"Кадр {frames}: {len(snapshot)} точек")

except KeyboardInterrupt:
    print("\nСтоп")
finally:
    running = False
    time.sleep(0.3)               
