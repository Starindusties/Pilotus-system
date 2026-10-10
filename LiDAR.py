import sys
import time
import numpy as np
import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

PORT = '/dev/ttyUSB0'
BAUD = 460800
MAX_DIST = 6000


class LidarWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RPLIDAR C1 — live")
        self.resize(800, 800)

        # --- поле отрисовки (декартово, метры) ---
        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.setXRange(-MAX_DIST, MAX_DIST)
        self.plot.setYRange(-MAX_DIST, MAX_DIST)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setBackground('w')

        # сетка-круги каждые 1 метр
        for r in range(1000, MAX_DIST + 1, 1000):
            circle = QtWidgets.QGraphicsEllipseItem(-r, -r, r*2, r*2)
            circle.setPen(pg.mkPen((180, 180, 180), width=1))
            self.plot.addItem(circle)

        # крестовина
        self.plot.addLine(x=0, pen=pg.mkPen((200, 200, 200)))
        self.plot.addLine(y=0, pen=pg.mkPen((200, 200, 200)))

        self.setCentralWidget(self.plot)

        # --- слой точек ---
        self.scatter = pg.ScatterPlotItem(
            size=4,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(255, 0, 0, 200)
        )
        self.plot.addItem(self.scatter)

        # --- открываем порт ---
        self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0xA5, 0x20]))
        print("Ответ:", self.ser.read(7).hex())

        # --- буфер ---
        self.points = []
        self.frames = 0

        # --- таймер чтения: 100 раз в секунду ---
        self.read_timer = QtCore.QTimer()
        self.read_timer.timeout.connect(self.read_serial)
        self.read_timer.start(10)

        # --- таймер отрисовки: 10 раз в секунду ---
        self.draw_timer = QtCore.QTimer()
        self.draw_timer.timeout.connect(self.redraw)
        self.draw_timer.start(100)

    def read_serial(self):
        n = self.ser.in_waiting
        if n < 5:
            return
        data = self.ser.read(n - (n % 5))
        for i in range(0, len(data), 5):
            chunk = data[i:i+5]
            if len(chunk) != 5:
                continue
            b0, b1, b2, b3, b4 = chunk
            if (b0 & 0x01) != ((b0 >> 1) & 0x01):
                continue
            quality = b0 >> 2
            angle = ((b1 >> 1) | (b2 << 7)) / 64.0
            distance = (b3 | (b4 << 8)) / 4.0
            if 100 < distance < MAX_DIST and quality > 20:
                self.points.append((angle, distance))

    def redraw(self):
        if not self.points:
            return
        arr = np.array(self.points)
        angles_rad = np.radians(arr[:, 0])
        dists = arr[:, 1]
        x = dists * np.sin(angles_rad)
        y = dists * np.cos(angles_rad)
        self.scatter.setData(x=x, y=y)
        self.frames += 1
        self.setWindowTitle(f"RPLIDAR C1 — кадр {self.frames}, точек: {len(arr)}")
        print(f"Кадр {self.frames}: {len(arr)} точек")
        self.points = []

    def closeEvent(self, event):
        try:
            self.ser.write(bytes([0xA5, 0x25]))
            self.ser.close()
        except:
            pass
        event.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = LidarWindow()
    win.show()
    sys.exit(app.exec_())
