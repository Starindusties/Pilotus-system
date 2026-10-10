 import asyncio
import numpy as np
import matplotlib.pyplot as plt
from rplidarc1 import RPLidar

# --- Настройки ---
PORT = "/dev/ttyUSB0"
BAUD = 460800
MAX_DIST = 6000   # 6 метров, как в RoboStudio

async def main():
    lidar = RPLidar(PORT, BAUD)

    # --- Настройка окна ---
    plt.ion()
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, MAX_DIST)
    ax.set_yticks([1000, 2000, 3000, 4000, 5000, 6000])
    ax.set_title('RPLIDAR C1 — до 6 м')
    scatter = ax.scatter([], [], s=8, c='red')
    fig.canvas.draw()
    fig.canvas.flush_events()

    print("Начинаю сканирование...")

    frames = 0

    while True:
        # Очищаем словарь перед новым кадром
        lidar.stop_event.clear()
        if lidar.output_dict is not None:
            lidar.output_dict.clear()

        # Запускаем сбор данных в фоне
        scan_task = asyncio.create_task(
            lidar.simple_scan(make_return_dict=True)
        )

        # Ждём 0.5 сек — этого хватит на несколько оборотов лидара
        await asyncio.sleep(0.5)

        # Останавливаем сбор
        lidar.stop_event.set()

        # Даём задаче завершиться
        try:
            await asyncio.wait_for(scan_task, timeout=1.0)
        except (asyncio.TimeoutError, Exception):
            pass

        # Проверяем, есть ли данные
        if not lidar.output_dict:
            print("Данных пока нет...")
            await asyncio.sleep(0.1)
            continue

        # --- Достаём точки из словаря ---
        angles = []
        dists = []
        for angle_deg, data in lidar.output_dict.items():
            d = data.get('d_mm', 0)
            q = data.get('q', 0)
            if 100 < d < MAX_DIST and q > 20:
                angles.append(np.radians(angle_deg))
                dists.append(d)

        # --- Обновляем график ---
        if angles:
            scatter.set_offsets(np.column_stack([angles, dists]))
            frames += 1
            ax.set_title(f'RPLIDAR C1 — кадр {frames}, точек: {len(angles)}')
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            print(f"Кадр {frames}: {len(angles)} точек")

        await asyncio.sleep(0.05)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nОстановлено пользователем.")
