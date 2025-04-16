#include "../include/Ransac.h"


namespace RansacNamespace{


/**
 * x_y_to_polynom - функция для преобразования координат точек в уравнения линий в форме полинома.
 *
 *  coord_for_lines - вектор контуров, представленных как векторы точек.
 *  Вектор линий (TLine2D), представленных в форме полиномов.
 */
    TL x_y_to_polynom(std::vector<std::vector<cv::Point>>& coord_for_lines) {
        TL Polylines; // Вектор для хранения уравнений линий.

        for (auto &coord_for_line : coord_for_lines) {
            CVD x = {};
            CVD y = {};

            // Преобразуем координаты точек из формата (x, y) в (y, x).
            for (auto &xy: coord_for_line) {
                x.push_back(xy.y);
                y.push_back(xy.x);
            }

            mrpt::math::TLine2D line = {};

            if (!x.empty()) { // Проверяем, что векторы x и y не пустые.
                int n = x.size(); // Количество точек.
                size_t m = 2; // Степень полинома (квадратичный).

                // Инициализируем матрицу A и вектор B для системы уравнений.
                std::vector<std::vector<double>> A(m + 1, std::vector<double>(m + 1, 0));
                std::vector<double> B(m + 1, 0);

                // Заполняем матрицу A и вектор B на основе координат x и y точек.
                for (size_t i = 0; i < static_cast<size_t>(n); i++) {
                    double xi = x[i], yi = y[i];
                    for (size_t j = 0; j <= m; j++) {
                        for (size_t k = 0; k <= m; k++) {
                            if ((j == 0) & (k == 0)) {
                                A[j][k] += 1;
                            } else {
                                A[j][k] += pow(xi, j + k);
                            }
                        }
                        B[j] += pow(xi, j) * yi;
                    }
                }

                // Решаем систему линейных уравнений методом Гаусса.
                for (size_t i = 0; i <= m; i++) {
                    double d = A[i][i];
                    for (size_t j = i; j <= m; j++) {
                        A[i][j] /= d;
                    }
                    B[i] /= d;
                    for (size_t j = i + 1; j <= m; j++) {
                        double d2 = A[j][i];
                        for (size_t k = i; k <= m; k++) {
                            A[j][k] -= d2 * A[i][k];
                        }
                        B[j] -= d2 * B[i];
                    }
                }

                // Вычисляем коэффициенты полинома и формируем уравнение линии.
                std::vector<double> res(m + 1);
                for (size_t i = m; static_cast<int>(i) >= 0; i--) {
                    for (size_t j = i + 1; j <= m; j++) {
                        B[i] -= A[i][j] * res[j];
                    }
                    res[i] = B[i];
                }

                // Создаем TLine2D с полученными коэффициентами полинома.
                if ((abs(res[2]) < 0.0001))
                    line = {res[2], res[1], res[0]}; // Уравнение линии в форме полинома.
                else
                    line = {0, 0, 0}; // Уравнение линии не найдено (линия параллельна оси X).
            }

            Polylines.push_back(line); // Добавляем уравнение линии в вектор.

        }

        return Polylines; // Возвращаем вектор уравнений линий в форме полиномов.
    }





/**
 * find_x_y - функция для поиска координат контуров и определения типа линий.
 *
 *  lines - вектор линий, к которым производится поиск контуров.
 *  contours - вектор контуров, представленных как векторы точек.
 *  width - ширина, используемая для определения близких контуров к линиям.
 *  result_coord - результирующий вектор контуров, разбитый по линиям.
 *  result_type_of_lines - вектор, в который будет записан результат определения типа линий.
 */
    void find_x_y(TL &lines, std::vector<std::vector<cv::Point>>& contours, double width,
                  std::vector<std::vector<cv::Point>>& result_coord, std::vector<bool>& result_type_of_lines) {

        result_coord.resize(lines.size()); // Инициализируем вектор результатов для контуров.
        std::vector<int> count_contours_in_line(result_coord.size());

        for (auto &cnt: contours) {
            cv::Rect boundingRect = cv::boundingRect(cnt);
            mrpt::math::TPoint2D center;
            center.y = boundingRect.x + (boundingRect.width / 2);
            center.x = boundingRect.y + (boundingRect.height / 2);

            for (size_t i = 0; i < lines.size(); i++) {
                if (abs(lines[i].distance(center)) < width) {
                    // Добавляем контур к результатам и увеличиваем счетчик контуров в линии.
                    result_coord[i].insert(result_coord[i].begin(), cnt.begin(), cnt.end());
                    count_contours_in_line[i]++;
                    break;
                }
            }
        }

        return_type_of_line(count_contours_in_line, result_type_of_lines); // Определяем типы линий.
    }

}

