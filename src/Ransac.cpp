#include "../include/Ransac.h"

using namespace mrpt;

using namespace mrpt::math;

using namespace mrpt::system;

using namespace std;



/**
 * RANSACLines - функция для обнаружения линий в наборе точек методом RANSAC.
 *
 *  coords - входной вектор точек, на которых будет выполняться поиск линий.
 *  min_inliers - минимальное количество точек, необходимое для определения линии (по умолчанию: 200).
 *  DIST_THRESHOLD - пороговое расстояние для RANSAC (по умолчанию: 0.3).
 *  Вектор обнаруженных линий типа TLine2D.
 */
RansacNamespace::TL RansacNamespace::RANSACLines(std::vector<cv::Point>& coords, size_t min_inliers, double DIST_THRESHOLD) {
    RansacNamespace::CVD x;
    RansacNamespace::CVD y;

    // Преобразуем координаты точек из формата (x, y) в (y, x).
    for (auto &xy: coords) {
        x.push_back(xy.y);
        y.push_back(xy.x);
    }

    std::vector<TLine2D> lines; // Вектор для хранения обнаруженных линий.
    vector<pair<size_t, TLine2D>> detectedLines; // Вектор пар, где первый элемент - количество точек, второй - линия.

    // Вызываем функцию ransac_detect_2D_lines для обнаружения линий.
    ransac_detect_2D_lines(x, y, detectedLines, DIST_THRESHOLD, min_inliers);

    // Переносим обнаруженные линии в вектор lines.
    for (auto p = detectedLines.begin(); p != detectedLines.end(); p++ ){
        lines.push_back(p->second);
    }

    return lines;
}


