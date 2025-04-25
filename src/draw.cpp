#include "../include/Ransac.h"


namespace RansacNamespace {



    /**
 * Преобразует параметры линии в координаты.
 *
 *  line      Линия в формате TLine2D, содержащая коэффициенты (a, b, c).
 *  cols      Количество точек, которые требуется вычислить.
 *  Polynom   Флаг, указывающий, является ли линия полиномом (true) или прямой (false).
 *
 *           Возвращает вектор координат x и y в формате {x, y}.
 */
    std::vector<std::vector<double>> param_to_coord(mrpt::math::TLine2D line, size_t rows, bool Polynom) {
        // Создаем векторы для хранения координат x и y.
        std::vector<double> x, y;
        double znach;

        // Проходим через каждую точку в диапазоне от 0 до cols.
        for (size_t i = 0; i < rows; i++) {
            if (Polynom) {
                // Если линия - полином, вычисляем координату y по формуле полинома.
                znach = line.coefs[0] * static_cast<double>(i) * static_cast<double>(i)
                        + line.coefs[1] * static_cast<double>(i)
                        + line.coefs[2];
            } else {
                // Если линия - прямая, вычисляем координату y по формуле прямой.
                znach = -(line.coefs[1] * static_cast<double>(i) + line.coefs[2]) / line.coefs[0];
            }

            // Проверяем, что координаты в пределах [0, 720].
            if ((abs(znach) <= 720) && (abs(znach) >= 0)) {
                // Добавляем координаты в соответствующие векторы x и y.
                x.push_back(static_cast<double>(i));
                y.push_back(znach);
            }
        }

        // Возвращаем вектор координат в зависимости от флага Polynom.
        if (Polynom)
            return {y, x}; // Возвращаем {y, x} для полинома.
        else
            return {x, y}; // Возвращаем {x, y} для прямой.
    }


/**
 * Рисует линии на изображении в соответствии с параметрами линий.
 *
 *  image     Изображение (матрица), на котором будут отрисованы линии.
 *  lines     Вектор линий в формате TLine2D, которые нужно отрисовать.
 *  Polynom   Флаг, указывающий, являются ли линии полиномами (true) или прямыми (false).
 *  result_type_of_lines   Указатель на вектор с значениями типов линий 1-сплошная 0-прерывистая.
 */
    void draw_lines(cv::Mat image, TL &lines, bool Polynom, std::vector<bool>& result_type_of_lines) {
        if(image.channels()==1) cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);

        vec coords_for_drow; // Вектор для хранения координат линий для отрисовки.
        std::vector<cv::Point> contour; // Вектор для хранения контуров линий.
        const cv::Point *pts; // Указатель на массив точек контура.

        cv::Scalar color; // Цвет линий.
        if (!Polynom)
            color = cv::Scalar(0, 0, 255); // Красный цвет для прямых.
        int npts;

        // Проходим по всем линиям вектора lines.
        for (size_t j = 0; j< lines.size(); j++) {
            mrpt::math::TLine2D line = lines[j];
            bool type = result_type_of_lines[j]; // получаем тип линии

            // Проверяем, что коэффициент c линии не близок к нулю (линия не вертикальная).
            if ((line.coefs[2] < -0.0001) || (line.coefs[2] > 0.0001)) {
                // Вычисляем координаты для отрисовки линии.
                coords_for_drow = param_to_coord(line, static_cast<size_t>(image.rows), Polynom);

                // Заполняем контур точками для отрисовки.
                for (size_t i = 0; i < coords_for_drow[0].size(); i++)
                    contour.emplace_back(coords_for_drow[0][i], coords_for_drow[1][i]);

                // Преобразуем контур в указатель на массив точек и определяем количество точек.
                pts = reinterpret_cast<const cv::Point*>(cv::Mat(contour).data);
                npts = cv::Mat(contour).rows;
                if (Polynom) {
                    // Выбирает цвет линии от значения в векторе resul_type_for_lines
                    if (type) {
                        color = {0, 0, 255};  // красный - сплошная
                    }
                    else
                        color = {0, 255, 0}; // зелёный - прерывистая
                }
                // Отрисовываем линию на изображении.
                polylines(image, &pts, &npts, 1, false, color, 3, cv::LINE_8);
                contour.clear(); // Очищаем контур для следующей линии.
            }
        }
    }


/**
 * draw_inliers - функция для рисования точек-внутренних элементов на изображении.
 *
 *  image - изображение, на котором будут отображены точки.
 *  coord - вектор координат точек, которые нужно нарисовать.
 */
    void draw_inliers(cv::Mat image, std::vector<cv::Point>& coord) {
        if (!coord.empty()) { // Проверяем, что вектор coord не пустой.
            for (auto point : coord) {
                // Рисуем круговую метку в заданной точке.
                cv::circle(image, point, 1, cv::Scalar(255, 0, 0), 3); // Синий цвет точки.
            }
        }
    }



}
