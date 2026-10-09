import asyncio
import numpy as np
import matplotlib.pyplot as plt
from rplidarc1 import RPLidar

# --- Настройки ---
PORT = "/dev/ttyUSB0"
BAUD = 460800
DELAY = 0.1  # пауза между обновлениями графика (сек)

async def main():
    lidar = RPLidar(PORT, BAUD)

    # --- Настройка окна matplotlib ---
    plt.ion()  # Включаем интерактивный режим (без блокировки)
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
    ax.set_theta_zero_location('N')  # Ноль сверху
    ax.set_theta_direction(-1)        # По часовой стрелке
    ax.set_ylim(0, 12000)             # Максимальная дистанция 12 метров
    ax.set_title('RPLIDAR C1 — нажми Ctrl+C для выхода')
    scatter = ax.scatter([], [], s=5, c='red')
    fig.canvas.draw()
    fig.canvas.flush_events()

    print("Начинаю сканирование...")

    while True:
        # Очищаем словарь для нового кадра
        lidar.output_dict.clear()
        lidar.stop_event.clear()

        # Запускаем сбор данных в фоне
        scan_task = asyncio.create_task(lidar.simple_scan(make_return_dict=True))
        
        # Ждём, пока соберётся достаточно данных (0.5 сек)
        await asyncio.sleep(0.5)
        
        # Останавливаем сбор и ждём завершения задачи
        lidar.stop_event.set()
        try:
            await asyncio.wait_for(scan_task, timeout=1.0)
        except (asyncio.TimeoutError, Exception):
            pass

        # --- Извлекаем точки из словаря ---
        angles, dists = [], []
        for angle_deg, data in lidar.output_dict.items():
            d = data.get('d_mm', 0)
            q = data.get('q', 0)
            # Фильтруем нулевые значения и очень далёкие точки
            if d > 100 and q > 0:
                angles.append(np.radians(angle_deg))  # Переводим в радианы для matplotlib
                dists.append(d)

        # --- Обновляем график ---
        if angles:
            scatter.set_offsets(np.column_stack([angles, dists]))
            ax.set_title(f'RPLIDAR C1 — точек: {len(angles)}')
            fig.canvas.draw()
            fig.canvas.flush_events()
            print(f"Обновлено точек: {len(angles)}")

        await asyncio.sleep(DELAY)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nОстановлено пользователем")
