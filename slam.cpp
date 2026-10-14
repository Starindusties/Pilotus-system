#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <vector>
#include <cmath>

using namespace sl;
using namespace cv;

const int   WIDTH      = 800;       // размер окна
const int   HEIGHT     = 800;
const float MAX_DIST   = 6000.0f;   // 6 метров
const float SCALE      = (WIDTH / 2.0f) / MAX_DIST; // мм → пиксели

int main(int argc, char* argv[])
{
    // --- Подключение ---
    Result<IChannel*> channelResult = createSerialPortChannel("/dev/ttyUSB0", 460800);
    IChannel* channel = *channelResult;

    Result<ILidarDriver*> lidarResult = createLidarDriver();
    ILidarDriver* lidar = *lidarResult;

    auto res = lidar->connect(channel);
    if (SL_IS_FAIL(res)) {
        fprintf(stderr, "Connect failed: %08x\n", (unsigned)res);
        delete lidar; delete channel;
        return -1;
    }

    // --- Info ---
    sl_lidar_response_device_info_t deviceInfo;
    res = lidar->getDeviceInfo(deviceInfo);
    if (SL_IS_OK(res)) {
        printf("Model: %d, FW: %d.%d, HW: %d\n",
               deviceInfo.model,
               deviceInfo.firmware_version >> 8,
               deviceInfo.firmware_version & 0xff,
               deviceInfo.hardware_version);
    }

    // --- Запуск ---
    bool support = false;
    lidar->checkMotorCtrlSupport(support);
    if (support) lidar->startMotor();
    lidar->startScan(0, 1);
    printf("Окно открыто. Нажми 'q' или ESC для выхода.\n");

    // --- Окно OpenCV ---
    namedWindow("RPLIDAR C1", WINDOW_AUTOSIZE);

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    while (true) {
        res = lidar->grabScanDataHq(nodes, count);
        if (SL_IS_FAIL(res)) continue;

        lidar->ascendScanData(nodes, count);

        // --- Чёрный холст ---
        Mat canvas(HEIGHT, WIDTH, CV_8UC3, Scalar(0, 0, 0));

        // --- Сетка: круги каждые 1 метр ---
        Point center(WIDTH / 2, HEIGHT / 2);
        for (int r = 1000; r <= (int)MAX_DIST; r += 1000) {
            circle(canvas, center, (int)(r * SCALE), Scalar(50, 50, 50), 1);
        }
        // --- Крестовина ---
        line(canvas, Point(center.x, 0),        Point(center.x, HEIGHT), Scalar(50, 50, 50), 1);
        line(canvas, Point(0, center.y),        Point(WIDTH, center.y),  Scalar(50, 50, 50), 1);

        // --- Точки ---
        int valid = 0;
        for (size_t i = 0; i < count; ++i) {
            float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
            float dist  = nodes[i].dist_mm_q2 / 4.0f;
            float qual  = nodes[i].quality;

            if (dist < 100 || dist > MAX_DIST || qual < 20) continue;

            float rad = angle * M_PI / 180.0f;
            int x = (int)(center.x + dist * SCALE * sin(rad));
            int y = (int)(center.y - dist * SCALE * cos(rad));

            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) continue;

            // Цвет зависит от расстояния: ближе — краснее, дальше — синее
            Scalar color;
            if (dist < 2000)      color = Scalar(0, 0, 255);      // красный
            else if (dist < 4000) color = Scalar(0, 200, 200);    // жёлтый
            else                  color = Scalar(255, 100, 0);    // синий

            circle(canvas, Point(x, y), 2, color, -1);
            valid++;
        }

        // --- Подписи ---
        char text[128];
        snprintf(text, sizeof(text), "Toчек: %d", valid);
        putText(canvas, text, Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
        putText(canvas, "1m / 2m / ... / 6m", Point(10, HEIGHT - 15),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(150, 150, 150), 1);

        imshow("RPLIDAR C1", canvas);

        int key = waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;   // q или ESC
    }

    // --- Останов ---
    lidar->stop();
    if (support) lidar->setMotorSpeed(0);
    destroyAllWindows();

    delete lidar;
    delete channel;
    return 0;
}
