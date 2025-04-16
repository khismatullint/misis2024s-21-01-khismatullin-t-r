#include "../include/Ransac.h"

namespace RansacNamespace{


/**
 * Возвращает координаты точки на нижней части линии в заданной строке `y`.
 *
 * @param line Линия (в виде mrpt::math::TLine2D), для которой нужно определить координаты точки.
 * @param y Координата `y`, на основе которой будет определена координата `x`.
 * @return cv::Point Точка с координатами x и y на линии.
 */
    cv::Point return_xy_low_point(mrpt::math::TLine2D &line, int y) {
        cv::Point minmax;
        minmax.y = y;
        // Расчет координаты x для заданной строки y на основе уравнения линии.
        minmax.x = static_cast<int>(line.coefs[0]*y*y + line.coefs[1]*y + line.coefs[2]);
        return minmax;
    }


/**
 * Создает и возвращает вектор из трех точек на линии `line`, распределенных по x1, x2, x3.
 *
 * @param x1 Первая координата x.
 * @param x2 Вторая координата x.
 * @param x3 Третья координата x.
 * @param line Линия (в виде mrpt::math::TLine2D), для которой нужно определить три точки.
 * @return std::vector<cv::Point> Вектор, содержащий три точки на линии.
 */
    std::vector<cv::Point> three_dots_l_r(int x1, int x2, int x3, mrpt::math::TLine2D &line) {
        std::vector<cv::Point> points;
        // Получаем три точки на линии, соответствующие заданным координатам x1, x2, x3.
        points.push_back(return_xy_low_point(line, x1));
        points.push_back(return_xy_low_point(line, x2));
        points.push_back(return_xy_low_point(line, x3));

        return points;
    }


/**
 * Возвращает три вектора точек в координатах изображения после преобразования перспективы.
 *
 * Функция принимает вектор векторов точек `points` и выполняет преобразование перспективы для этих точек.
 * Используются параметры `parametersBird` из настроек (`init.parametersBird`) для выполнения преобразования.
 *
 * points - Вектор векторов точек для преобразования.
 */
    void return_three_vec_point_in_img_coord(std::vector<std::vector<cv::Point>>& points) {
        // Инициализация настроек
        settings init;
        // Получение параметров для преобразования
        auto parameters = init.parametersBird;
        // Создание векторов для исходных и преобразованных точек
        std::vector<cv::Point2f> srcPoints;
        std::vector<cv::Point2f> dstPoints;
        // Заполнение вектора исходных точек
        for (const auto& vec : points) {
            for (const auto& point : vec) {
                srcPoints.emplace_back(point);
            }
        }
        // Определение исходных и конечных точек для преобразования перспективы
        cv::Point2f dst[4] = {cv::Point2i(parameters[0], parameters[1]),
                              cv::Point2i(parameters[2], parameters[3]),
                              cv::Point2i(parameters[4], parameters[5]),
                              cv::Point2i(parameters[6], parameters[7])};
        cv::Point2f src[4] = {cv::Point2i(0, 0),
                              cv::Point2i(parameters[9], 0),
                              cv::Point2i(0, parameters[8]),
                              cv::Point2i(parameters[9], parameters[8])};

        // Получение матрицы преобразования перспективы
        cv::Mat M = cv::getPerspectiveTransform(src, dst);
        // Преобразование точек
        cv::perspectiveTransform(srcPoints, dstPoints, M);
        // Группировка преобразованных точек обратно в вектор std::vector<std::vector<cv::Point>>
        std::vector<std::vector<cv::Point>> transformedPoints(points.size());
        points.clear();

        // Распределение преобразованных точек по векторам
        transformedPoints[0].push_back(dstPoints[0]);
        transformedPoints[0].push_back(dstPoints[1]);
        transformedPoints[0].push_back(dstPoints[2]);
        transformedPoints[1].push_back(dstPoints[3]);
        transformedPoints[1].push_back(dstPoints[4]);
        transformedPoints[1].push_back(dstPoints[5]);

        points = transformedPoints; // Присваивание преобразованных точек исходному вектору
    }


/**
 *  Находит расстояние от точки до центра в новой системе координат.
 *
 * Функция вычисляет расстояние от переданной точки до центра в новой системе координат.
 * Для этого производится преобразование координат переданной точки в новую систему с помощью матрицы преобразования.
 * Затем вычисляется расстояние между центром и точкой в новой системе координат.
 *
 *  Center Центр в исходной системе координат типа cv::Point2d.
 *  point Точка для вычисления расстояния типа cv::Point.
 *  Расстояние до центра в новой системе координат типа cv::Point2d.
 */
    cv::Point2d find_distance_point_to_center(cv::Point2d Center, cv::Point point) {
        // Инициализация настроек
        settings init;

        // Создание векторов для точки и центра
        Eigen::Vector3d WC;
        Eigen::Vector3d point_eigen;
        // Задание координат точки в виде вектора Eigen
        point_eigen << point.x, point.y, 1.0;
        // Преобразование координат точки в новую систему с помощью матрицы преобразования
        point_eigen = init.transformationMatrix * point_eigen;
        // Задание координат центра в виде вектора Eigen
        WC << Center.x, Center.y, 1.0;

        // Вычисление расстояния по осям x и y между центром и точкой в новой системе координат
        double distance_y = abs(WC(1) - point_eigen(1));
        double distance_x = WC(0) - point_eigen(0);
        // Создание объекта cv::Point2d с расстояниями по осям x и y
        cv::Point2d dist(distance_x, distance_y);

        return dist; // Возврат расстояния до центра в новой системе координат
    }


/**
 * Расчитывает крайние левую и правую точки  преобразует точки из координат камеры в мировые
 *
 * @param lines Вектор прямых линий, определяющих контуры дороги.
 * @param image Исходное изображение, на котором будет нарисовано пространство дороги.
 * @param transformationMatrix Матрица преобразования для перехода в мировые координаты.
 * @param left_right_distance Вектор расстояний до левой и правой границ дороги.
 * @param Three_points_to_line Вектор векторов точек для каждой из линий.
 */
    std::vector<std::vector<cv::Point2d>> get_three_point_vector(TL lines, const cv::Mat &image, Eigen::Matrix3d& transformationMatrix, std::vector<double>& left_right_distance) {

        // Объявление переменных для хранения расстояний и координат
        double left_distance, right_distance;
        int img_h = image.rows;
        left_distance = left_right_distance[0];  // Левое расстояние до границы дороги
        right_distance = left_right_distance[1]; // Правое расстояние до границы дороги
        size_t r_i, l_i;  // Индексы левой и правой линий дороги
        Eigen::Vector3d worldCoordinates1, worldCoordinates2, worldCenter;  // Мировые координаты точек
        std::vector<std::vector<cv::Point>> minmax_vector;  // Вектор для хранения точек
        Eigen::Vector3d Point_on_image_left, Point_on_image_right;  // Точки на изображении
        int Center_img = static_cast<int>(floor(image.cols / 2)); // Координата центра изображения по горизонтали
        std::vector<std::vector<cv::Point>> buf;
        std::vector<std::vector<cv::Point>> Three_points_to_line;
        std::vector<std::vector<cv::Point2d>> retThree_points_to_line;

        // Проверка наличия линий
        if (!lines.empty()) {
            r_i = 0;
            // Находим точку на нижней части линии слева от центра изображения
            cv::Point xy2 = return_xy_low_point(lines[r_i], img_h);
            while (((xy2.x < Center_img)) && (r_i < lines.size())) {
                r_i++;
                xy2 = return_xy_low_point(lines[r_i], img_h);
            }

            // Обработка выхода за границы вектора линий
            if (r_i > lines.size()-1){
                r_i--;
                xy2 = return_xy_low_point(lines[r_i], img_h);
            }

            // Находим точку на нижней части линии справа от центра изображения
            l_i = lines.size()-1;
            cv::Point xy1 = return_xy_low_point(lines[l_i], img_h);
            while (((xy1.x > Center_img) || (xy1.x == 0))) {
                if (static_cast<int>(l_i) - 1 >= 0) {
                    l_i--;
                } else {
                    xy1 = return_xy_low_point(lines[l_i], img_h);
                    break;
                }
                xy1 = return_xy_low_point(lines[l_i], img_h);
            }

            // Рисуем r точку в центре изображения
            cv::circle(image, {Center_img, image.rows}, 20, cv::Scalar(0, 0, 255), 10);
            // Задаем координаты точек для левой и правой границы дороги на изображении

            buf.push_back({xy1});
            buf.push_back({xy2});
            buf.push_back({cv::Point(Center_img, img_h)});

            return_three_vec_point_in_img_coord(buf);
            Point_on_image_left<< static_cast<double>(buf[0][0].x), static_cast<double>(buf[0][0].y), 1.0;
            Point_on_image_right<< static_cast<double>(buf[0][1].x), static_cast<double>(buf[0][1].y), 1.0;
        }

        // Задаем мировые координаты для центра изображения и точек на изображении
        worldCenter << buf[0][2].x, buf[0][2].y , 1.0;
        worldCenter = transformationMatrix * worldCenter;

        worldCoordinates1 = transformationMatrix * Point_on_image_left;
        worldCoordinates2 = transformationMatrix * Point_on_image_right;

        // Вычисляем ширину дороги с помощью мировых координат
        left_distance = sqrt(pow(worldCoordinates1(0) - worldCenter(0), 2) + pow(worldCoordinates1(1) - worldCenter(1), 2));
        right_distance = sqrt(pow(worldCenter(0) - worldCoordinates2(0), 2) + pow(worldCenter(1) - worldCoordinates2(1), 2));
        left_right_distance = {left_distance, right_distance}; // Обновляем расстояния

        // Очищаем вектор и добавляем три точки для каждой из линий дороги
        Three_points_to_line.clear();
        Three_points_to_line.push_back(three_dots_l_r(img_h, img_h/2, 0, lines[l_i]));
        Three_points_to_line.push_back(three_dots_l_r(img_h, img_h/2, 0, lines[r_i]));

        return_three_vec_point_in_img_coord(Three_points_to_line);

        std::vector<cv::Point2d> dist;

        buf.clear();
        for (double i=1; i>=0; i-=(0.5))
            buf.push_back({cv::Point(Center_img, static_cast<int>(img_h*i))});

        return_three_vec_point_in_img_coord(buf);

        for (size_t j = 0; j<2 ; j++) {
            dist.clear();
            double image_rows = image.rows;
            for (size_t i = 0; i != 3; i++) {
                dist.push_back(find_distance_point_to_center(cv::Point2d(worldCenter(0), worldCenter(1)),
                                                             Three_points_to_line[j][i]));
                image_rows -= image.rows / 2;
                worldCenter << buf[0][0].x, buf[0][0].y, 1.0;
                worldCenter = transformationMatrix * worldCenter;
            }
            retThree_points_to_line.push_back(dist);
        }
        dist.clear();
        std::cout<<"\n\nleft points: ";
        for(auto j : retThree_points_to_line[0]){
            std::cout<<j<<"  ||   ";
        }
        std::cout<<"\nright points: ";
        for(auto j : retThree_points_to_line[1]){
            std::cout<<j<<"  ||   ";
        }
        std::cout<<"\n\n";
        return retThree_points_to_line;
    }

}