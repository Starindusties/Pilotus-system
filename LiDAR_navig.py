import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# --- Настройки ---
PORT = '/dev/ttyUSB0'
BAUD = 460800

# --- Открываем порт ---
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(0.5)
ser.reset_input_buffer()

# --- Запускаем сканирование ---
ser.write(bytes([0xA5, 0x20]))       # START_SCAN
resp = ser.read(7)
print("Ответ на START_SCAN:", resp.hex())

# --- Настройка графика ---
plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)
ax.set_ylim(0, 12000)
ax.set_title('RPLIDAR C1 — нажми Ctrl+C для выхода')
scatter = ax.scatter([], [], s=5, c='red')
fig.canvas.draw()
fig.canvas.flush_events()

print("Сканирование пошло. Ctrl+C для выхода.")

# --- Цикл чтения ---
scan_points = []

try:
    while True:
        data = ser.read(5)
        if len(data) != 5:
            continue

        b0, b1, b2, b3, b4 = data

        # Проверка старт-бита
        if (b0 & 0x01) != ((b0 >> 1) & 0x01):
            continue

        quality = b0 >> 2
        angle = ((b1 >> 1) | (b2 << 7)) / 64.0
        distance = (b3 | (b4 << 8)) / 4.0

        if distance > 100 and quality > 0:
            scan_points.append((angle, distance))

        # Как накопилось ~500 точек — рисуем
        if len(scan_points) >= 500:
            angles = np.radians([p[0] for p in scan_points])
            dists = [p[1] for p in scan_points]

            scatter.set_offsets(np.column_stack([angles, dists]))
            ax.set_title(f'RPLIDAR C1 — точек: {len(scan_points)}')
            fig.canvas.draw()
            fig.canvas.flush_events()

            scan_points = []   # очищаем буфер для нового кадра

except KeyboardInterrupt:
    print("\nОстановлено.")
finally:
    ser.write(bytes([0xA5, 0x25]))   # STOP
    ser.close()
    print("Порт закрыт.")
